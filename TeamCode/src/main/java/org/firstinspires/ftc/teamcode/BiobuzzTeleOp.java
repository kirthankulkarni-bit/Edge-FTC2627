package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * BIOBUZZ TeleOp: the Strafer drive plus intake, launcher and Limelight auto-aim.
 *
 * DRIVER (gamepad 1)
 *   Left stick            = drive / strafe
 *   Right stick X         = turn
 *   Right bumper (hold)   = slow mode (50%)
 *   Left bumper (hold)    = auto-aim: turns toward our raised CELL's AprilTags
 *
 * OPERATOR (gamepad 2)
 *   Right trigger (hold)  = intake (stops by itself at 4 POLLEN - rule G407)
 *   A                     = spin up launcher       X = stop launcher
 *   Right bumper          = launch everything we hold (waits until the flywheel is at speed)
 *   Dpad up / down        = fix the POLLEN count if the sensor missed one
 *
 * BEFORE START: gamepad 1 X = red, B = blue (defaults to the alliance of the last auto run).
 */
@TeleOp(name = "BIOBUZZ TeleOp", group = "BIOBUZZ")
public class BiobuzzTeleOp extends LinearOpMode {

    // ================= TUNING SECTION =================
    // Drive (same values as Strafer TeleOp)
    private static final double STRAFE_DIRECTION = -1.0;
    private static final double TURN_DIRECTION = 1.0;
    private static final double INPUT_GAIN = 1.25;
    private static final double STRAFE_BOOST = 1.15;
    private static final double SPEED_NORMAL = 1.0;
    private static final double SPEED_SLOW = 0.5;
    // Auto-aim: turn power per degree of tx, capped so it can't spin wildly
    private static final double AIM_KP = 0.02;          // TODO: tune; flip the sign if it turns away
    private static final double AIM_MAX_TURN = 0.4;
    private static final double CAMERA_TX_OFFSET_DEG = 0; // TODO: same value as in BiobuzzAuto
    // ==================================================

    private DcMotor leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        Intake intake = new Intake(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);
        TagAligner aligner = new TagAligner(hardwareMap);
        // The auto ends with an empty robot (it launches everything), so start TeleOp at 0.
        intake.setCount(0);

        boolean red = BiobuzzAuto.lastAllianceRed;
        while (opModeInInit()) {
            if (gamepad1.x) red = true;
            if (gamepad1.b) red = false;
            telemetry.addData("Alliance (X red / B blue)", red ? "RED" : "BLUE");
            telemetry.addData("Limelight", aligner.available() ? "found" : "not configured - no auto-aim");
            telemetry.update();
        }

        boolean lastUp = false, lastDown = false;
        boolean firing = false;

        while (opModeIsActive()) {
            // ---------------- DRIVE (gamepad 1)
            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x * STRAFE_DIRECTION * STRAFE_BOOST;
            double turn   =  gamepad1.right_stick_x * TURN_DIRECTION;

            // Auto-aim replaces the driver's turn while held, if a tag of OUR HIVE is visible.
            // Either of our CELLs: red 30-37, blue 38-45. The driver still controls drive/strafe.
            String aim = "off";
            if (gamepad1.left_bumper) {
                double tx = red ? aligner.txTo(30, 37) : aligner.txTo(38, 45);
                if (!Double.isNaN(tx)) {
                    turn = clamp(AIM_KP * (tx - CAMERA_TX_OFFSET_DEG), AIM_MAX_TURN);
                    aim = String.format("tx %.1f", tx);
                } else {
                    aim = "no tag";
                }
            }

            double speed = gamepad1.right_bumper ? SPEED_SLOW : SPEED_NORMAL;
            double lf = (drive + strafe + turn) * INPUT_GAIN;
            double lb = (drive - strafe + turn) * INPUT_GAIN;
            double rf = (drive - strafe - turn) * INPUT_GAIN;
            double rb = (drive + strafe - turn) * INPUT_GAIN;
            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(lf), Math.abs(lb)),
                    Math.max(Math.abs(rf), Math.abs(rb))));
            leftFront.setPower(lf / max * speed);
            leftBack.setPower(lb / max * speed);
            rightFront.setPower(rf / max * speed);
            rightBack.setPower(rb / max * speed);

            // ---------------- MECHANISMS (gamepad 2)
            // Intake: hold the trigger. The Intake class refuses to run once 4 are held.
            if (gamepad2.right_trigger > 0.3) {
                if (!intake.isRunning()) intake.start();
            } else {
                intake.stop();
            }

            if (gamepad2.a) launcher.spinUp();
            if (gamepad2.x) launcher.stop();
            if (gamepad2.right_bumper && !firing && intake.count() > 0) {
                intake.stop();
                launcher.fire(intake.count());
                firing = true;
            }
            // While firing, every piece that leaves the launcher is no longer held.
            if (firing) {
                intake.setCount(launcher.shotsLeft());
                if (launcher.isDone()) firing = false;
            }

            // Manual count fix, one step per press.
            if (gamepad2.dpad_up && !lastUp) intake.setCount(Math.min(Intake.MAX_PIECES, intake.count() + 1));
            if (gamepad2.dpad_down && !lastDown) intake.setCount(Math.max(0, intake.count() - 1));
            lastUp = gamepad2.dpad_up;
            lastDown = gamepad2.dpad_down;

            intake.update();
            launcher.update();

            telemetry.addData("Alliance", red ? "RED" : "BLUE");
            telemetry.addData("POLLEN held", "%d / %d%s", intake.count(), Intake.MAX_PIECES, intake.isFull() ? "  FULL" : "");
            telemetry.addData("Launcher", "%.0f RPM %s", launcher.rpm(), launcher.isReady() ? "READY" : "");
            telemetry.addData("Auto-aim", aim);
            telemetry.addData("Slow mode", gamepad1.right_bumper ? "ON" : "off");
            telemetry.update();
        }
        aligner.stop();
    }

    private static double clamp(double v, double limit) {
        return Math.max(-limit, Math.min(limit, v));
    }
}
