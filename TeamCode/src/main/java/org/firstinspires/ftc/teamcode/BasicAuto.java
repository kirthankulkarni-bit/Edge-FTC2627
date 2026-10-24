package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Basic autonomous for a goBILDA Strafer (mecanum) on a REV Control Hub.
 *
 * Routine (runs once, then stops):
 *   1. Drive forward 12 inches
 *   2. Turn around (180 degrees clockwise)
 *   3. Drive forward 12 inches
 */
@Autonomous(name = "Basic Auto", group = "Practice", preselectTeleOp = "Strafer TeleOp")
public class BasicAuto extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private final ElapsedTime runtime = new ElapsedTime();

    // ================= TUNING SECTION =================
    static final boolean USE_ENCODERS = false;
    static final double DRIVE_POWER = 0.5;
    static final double INCHES_PER_SECOND = 25.0;
    static final double TICKS_PER_MOTOR_REV = 537.7;
    static final double GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double TICKS_PER_INCH =
            (TICKS_PER_MOTOR_REV * GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double MOVE_TIMEOUT_SECONDS = 4.0;
    static final double SPIN_POWER = 0.5;
    static final double SECONDS_PER_ROTATION = 1.5;
    // ==================================================

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

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (USE_ENCODERS) {
            setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            setAllModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("Status", "Initialized. Press play.");
        telemetry.addData("Mode", USE_ENCODERS ? "Encoders" : "Timed driving");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            driveInches(12);
            sleep(500);
            turnClockwise(180);
            sleep(500);
            driveInches(12);

            setDrivePower(0, 0);
            telemetry.addData("Status", "Done");
            telemetry.update();
        }
    }

    private void driveInches(double inches) {
        if (USE_ENCODERS) {
            driveInchesEncoders(inches);
        } else {
            double seconds = Math.abs(inches) / INCHES_PER_SECOND;
            double power = inches >= 0 ? DRIVE_POWER : -DRIVE_POWER;
            driveForTime(power, power, seconds);
        }
    }

    private void turnClockwise(double degrees) {
        double seconds = (degrees / 360.0) * SECONDS_PER_ROTATION;
        driveForTime(SPIN_POWER, -SPIN_POWER, seconds);
    }

    private void driveForTime(double left, double right, double seconds) {
        runtime.reset();
        setDrivePower(left, right);
        while (opModeIsActive() && runtime.seconds() < seconds) {
            telemetry.addData("Timed move", "%.2f of %.2f s", runtime.seconds(), seconds);
            telemetry.update();
        }
        setDrivePower(0, 0);
    }

    private void driveInchesEncoders(double inches) {
        int ticks = (int) Math.round(inches * TICKS_PER_INCH);

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + ticks);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + ticks);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + ticks);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + ticks);

        setAllModes(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(DRIVE_POWER, DRIVE_POWER);

        runtime.reset();
        while (opModeIsActive()
                && runtime.seconds() < MOVE_TIMEOUT_SECONDS
                && leftFront.isBusy() && leftBack.isBusy()
                && rightFront.isBusy() && rightBack.isBusy()) {
            telemetry.addData("Target ticks", ticks);
            telemetry.update();
        }

        setDrivePower(0, 0);
        setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setDrivePower(double left, double right) {
        leftFront.setPower(left);
        leftBack.setPower(left);
        rightFront.setPower(right);
        rightBack.setPower(right);
    }

    private void setAllModes(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }
}
