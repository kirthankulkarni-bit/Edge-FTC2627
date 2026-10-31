package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Tunes the launcher flywheel's velocity PIDF on the real robot. Gamepad 1:
 *
 *   B (hold ~3 s)       = STEP 1: measure max speed at full power -> prints the F to use
 *   A                   = flywheel on / off at TARGET_RPM
 *   Dpad up / down      = pick what to change (P, I, D, F, target RPM)
 *   Dpad right / left   = increase / decrease it
 *   Right bumper        = launch ONE piece, then watch the dip and recovery time
 *
 * Goal: after a shot, RPM drops (the "dip") and climbs back. Raise P until recovery is fast
 * but the RPM doesn't bounce above and below the target. Add a little I only if it settles
 * slightly below target. Then copy the numbers into the TUNING SECTION of Launcher.java -
 * changes made here are NOT saved when the OpMode stops.
 */
@TeleOp(name = "Launcher Tuner", group = "Test")
public class LauncherTuner extends LinearOpMode {

    private static final String[] NAMES = {"P", "I", "D", "F", "Target RPM"};

    @Override
    public void runOpMode() {
        Launcher launcher = new Launcher(hardwareMap);
        DcMotorEx flywheel = hardwareMap.tryGet(DcMotorEx.class, "launcher");

        telemetry.addData("Flywheel", flywheel == null ? "NOT configured ('launcher') - nothing to tune" : "found");
        telemetry.addData("Start with", "B to measure max speed");
        telemetry.update();
        waitForStart();

        int selected = 0;
        boolean on = false;
        boolean lastA = false, lastUp = false, lastDown = false, lastLeft = false, lastRight = false, lastRb = false;

        // Shot measurement
        boolean watchingShot = false;
        double dipRpm = 0, recoverySeconds = 0;
        ElapsedTime shotTimer = new ElapsedTime();

        // Max-speed measurement
        double maxTicksPerSec = 0;
        ElapsedTime measureTimer = new ElapsedTime();
        boolean measuring = false;

        while (opModeIsActive()) {
            // ---- STEP 1: max speed (B). Full power, no speed control, for 3 seconds.
            if (gamepad1.b && flywheel != null && !measuring) {
                measuring = true;
                maxTicksPerSec = 0;
                measureTimer.reset();
                flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                flywheel.setPower(1.0);
            }
            if (measuring) {
                maxTicksPerSec = Math.max(maxTicksPerSec, flywheel.getVelocity());
                if (measureTimer.seconds() > 3.0 || !opModeIsActive()) {
                    flywheel.setPower(0);
                    flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    launcher.applyPIDF();
                    measuring = false;
                }
                telemetry.addData("Measuring max speed", "%.1f s ... %.0f ticks/s", measureTimer.seconds(), maxTicksPerSec);
                telemetry.update();
                continue;  // don't let the Launcher class fight the full-power test
            }

            // ---- Flywheel on/off (A)
            if (gamepad1.a && !lastA) {
                on = !on;
                if (on) launcher.spinUp(); else launcher.stop();
            }

            // ---- Pick and change a value (dpad)
            if (gamepad1.dpad_up && !lastUp) selected = (selected + NAMES.length - 1) % NAMES.length;
            if (gamepad1.dpad_down && !lastDown) selected = (selected + 1) % NAMES.length;
            if (gamepad1.dpad_right && !lastRight) change(selected, +1, launcher);
            if (gamepad1.dpad_left && !lastLeft) change(selected, -1, launcher);

            // ---- Fire one (right bumper) and start measuring the dip
            if (gamepad1.right_bumper && !lastRb && on && launcher.isReady()) {
                launcher.fire(1);
                watchingShot = true;
                dipRpm = launcher.rpm();
                recoverySeconds = 0;
                shotTimer.reset();
            }

            lastA = gamepad1.a;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            lastRb = gamepad1.right_bumper;

            launcher.update();

            double rpm = launcher.rpm();
            if (watchingShot) {
                dipRpm = Math.min(dipRpm, rpm);
                // Recovered = back within tolerance after the piece has left (give it 0.1 s to dip first).
                if (shotTimer.seconds() > 0.1 && launcher.isReady() && launcher.isDone()) {
                    recoverySeconds = shotTimer.seconds();
                    watchingShot = false;
                } else if (shotTimer.seconds() > 3) {
                    watchingShot = false;  // never recovered: P/F too low
                    recoverySeconds = Double.NaN;
                }
            }

            telemetry.addData("Flywheel (A)", on ? "ON" : "off");
            telemetry.addData("RPM", "%.0f / target %.0f  %s", rpm, Launcher.TARGET_RPM, launcher.isReady() ? "READY" : "");
            telemetry.addLine();
            for (int i = 0; i < NAMES.length; i++) {
                telemetry.addData((i == selected ? "> " : "  ") + NAMES[i], "%.4f", value(i));
            }
            telemetry.addLine();
            telemetry.addData("Last shot dip", "%.0f RPM (%.0f below target)", dipRpm, Launcher.TARGET_RPM - dipRpm);
            telemetry.addData("Last recovery", Double.isNaN(recoverySeconds) ? "did not recover in 3 s"
                    : String.format("%.2f s (SHOT_INTERVAL is %.2f s)", recoverySeconds, Launcher.SHOT_INTERVAL));
            if (maxTicksPerSec > 0) {
                telemetry.addData("Measured max", "%.0f ticks/s -> use F = %.2f, P = %.2f",
                        maxTicksPerSec, 32767 / maxTicksPerSec, 0.1 * 32767 / maxTicksPerSec);
            }
            telemetry.update();
        }
        launcher.stop();
        launcher.update();
    }

    private static double value(int i) {
        switch (i) {
            case 0: return Launcher.FLY_P;
            case 1: return Launcher.FLY_I;
            case 2: return Launcher.FLY_D;
            case 3: return Launcher.FLY_F;
            default: return Launcher.TARGET_RPM;
        }
    }

    /** Gains change by 10% per press (works at any scale); RPM by 100. */
    private static void change(int i, int dir, Launcher launcher) {
        double factor = dir > 0 ? 1.1 : 1 / 1.1;
        switch (i) {
            case 0: Launcher.FLY_P = bump(Launcher.FLY_P, factor, dir, 0.05); break;
            case 1: Launcher.FLY_I = bump(Launcher.FLY_I, factor, dir, 0.01); break;
            case 2: Launcher.FLY_D = bump(Launcher.FLY_D, factor, dir, 0.01); break;
            case 3: Launcher.FLY_F = bump(Launcher.FLY_F, factor, dir, 0.5); break;
            default:
                Launcher.TARGET_RPM = Math.max(0, Launcher.TARGET_RPM + 100 * dir);
                launcher.setTargetRpm(Launcher.TARGET_RPM);
                break;
        }
        launcher.applyPIDF();
    }

    /** Multiply by 10%; from 0, step up by `start` so a gain can be turned on. */
    private static double bump(double v, double factor, int dir, double start) {
        if (v == 0) return dir > 0 ? start : 0;
        double next = v * factor;
        return next < start / 10 ? 0 : next;
    }
}
