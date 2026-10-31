package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * Pit check: run this first whenever the robot is rebuilt or rewired.
 *
 *   - Lists every device our code expects and whether the robot configuration has it
 *   - Dpad up / down = pick a motor, hold A = spin it slowly forward, hold B = backward
 *     (each drive wheel should roll the robot FORWARD with A; if not, fix its direction)
 *   - Shows encoder counts, Pinpoint position, beam-break state, Limelight tags, battery
 *
 * Put the robot on a stand or blocks before spinning drive motors.
 */
@TeleOp(name = "Hardware Check", group = "Test")
public class HardwareCheck extends LinearOpMode {

    private static final double TEST_POWER = 0.3;

    // Every motor name used anywhere in our code.
    private static final String[] MOTORS = {
            "leftFront", "leftBack", "rightFront", "rightBack", "intake", "launcher", "feeder"
    };

    @Override
    public void runOpMode() {
        RobotUtil.enableBulkCaching(hardwareMap);

        List<String> names = new ArrayList<>();
        List<DcMotorEx> motors = new ArrayList<>();
        StringBuilder missing = new StringBuilder();
        for (String n : MOTORS) {
            DcMotorEx m = hardwareMap.tryGet(DcMotorEx.class, n);
            if (m != null) {
                names.add(n);
                motors.add(m);
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                missing.append(n).append(' ');
            }
        }
        DigitalChannel beam = hardwareMap.tryGet(DigitalChannel.class, "intakeBeam");
        if (beam != null) beam.setMode(DigitalChannel.Mode.INPUT);
        GoBildaPinpointDriver pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, "pinpoint");
        Limelight3A limelight = hardwareMap.tryGet(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.pipelineSwitch(TagAligner.PIPELINE);
            limelight.start();
        }

        telemetry.addData("Motors found", names.toString());
        telemetry.addData("Motors MISSING", missing.length() == 0 ? "none" : missing.toString());
        telemetry.addData("intakeBeam", beam != null ? "found" : "MISSING");
        telemetry.addData("pinpoint", pinpoint != null ? "found" : "MISSING");
        telemetry.addData("limelight", limelight != null ? "found" : "MISSING");
        telemetry.addData("Battery", "%.2f V", RobotUtil.batteryVoltage(hardwareMap));
        telemetry.addLine("Drive motors are required. The rest are optional until built.");
        telemetry.update();
        waitForStart();

        // Starting fresh makes it easy to check the Pinpoint: push the robot 24 in forward -> x ~ 24.
        if (pinpoint != null) pinpoint.resetPosAndIMU();

        int selected = 0;
        boolean lastUp = false, lastDown = false;
        ElapsedTime voltTimer = new ElapsedTime();
        double volts = RobotUtil.batteryVoltage(hardwareMap);

        while (opModeIsActive()) {
            if (!motors.isEmpty()) {
                if (gamepad1.dpad_up && !lastUp) selected = (selected + motors.size() - 1) % motors.size();
                if (gamepad1.dpad_down && !lastDown) selected = (selected + 1) % motors.size();
                for (int i = 0; i < motors.size(); i++) {
                    double p = 0;
                    if (i == selected && gamepad1.a) p = TEST_POWER;
                    if (i == selected && gamepad1.b) p = -TEST_POWER;
                    motors.get(i).setPower(p);
                }
            }
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            telemetry.addLine("Dpad = pick motor, hold A = forward, B = backward");
            for (int i = 0; i < motors.size(); i++) {
                DcMotorEx m = motors.get(i);
                telemetry.addData((i == selected ? "> " : "  ") + names.get(i),
                        "pos %d  vel %.0f t/s", m.getCurrentPosition(), m.getVelocity());
            }

            if (beam != null) telemetry.addData("intakeBeam", beam.getState() ? "clear" : "BROKEN (piece)");

            if (pinpoint != null) {
                pinpoint.update();
                telemetry.addData("Pinpoint x / y / heading", "%.1f in / %.1f in / %.1f deg",
                        pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH),
                        pinpoint.getHeading(AngleUnit.DEGREES));
                telemetry.addLine("  push forward -> x up, push left -> y up, turn left -> heading up");
            }

            if (limelight != null) {
                LLResult r = limelight.getLatestResult();
                if (r != null && r.isValid()) {
                    StringBuilder ids = new StringBuilder();
                    for (LLResultTypes.FiducialResult t : r.getFiducialResults()) ids.append(t.getFiducialId()).append(' ');
                    telemetry.addData("Limelight tags", ids.length() == 0 ? "target, no tag IDs" : ids.toString());
                    telemetry.addData("tx / ty", "%.1f / %.1f", r.getTx(), r.getTy());
                } else {
                    telemetry.addData("Limelight tags", "none in view");
                }
            }

            if (voltTimer.seconds() > 0.5) {
                volts = RobotUtil.batteryVoltage(hardwareMap);
                voltTimer.reset();
            }
            telemetry.addData("Battery", "%.2f V%s", volts, volts < RobotUtil.LOW_BATTERY_VOLTS ? "  LOW" : "");
            telemetry.update();
        }
        for (DcMotorEx m : motors) m.setPower(0);
        if (limelight != null) limelight.stop();
    }
}
