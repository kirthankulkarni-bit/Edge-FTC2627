package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Mecanum (goBILDA Strafer) drive. Tuned for speed.
 *
 * Controls (gamepad 1):
 *   Left stick  up/down    = drive forward / backward
 *   Left stick  left/right = strafe left / right
 *   Right stick left/right = turn
 *   Right bumper (hold)    = slow mode (50% speed)
 */
@TeleOp(name = "Strafer TeleOp", group = "Drive")
public class StraferTeleOp extends LinearOpMode {

    private static final double STRAFE_DIRECTION = -1.0;
    private static final double TURN_DIRECTION = 1.0;
    private static final double INPUT_GAIN = 1.25;
    private static final double STRAFE_BOOST = 1.15;
    private static final double SPEED_NORMAL = 1.0;
    private static final double SPEED_SLOW = 0.5;

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

        telemetry.addData("Status", "Initialized - press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x * STRAFE_DIRECTION * STRAFE_BOOST;
            double turn   =  gamepad1.right_stick_x * TURN_DIRECTION;

            double speed = gamepad1.right_bumper ? SPEED_SLOW : SPEED_NORMAL;

            double lf = (drive + strafe + turn) * INPUT_GAIN;
            double lb = (drive - strafe + turn) * INPUT_GAIN;
            double rf = (drive - strafe - turn) * INPUT_GAIN;
            double rb = (drive + strafe - turn) * INPUT_GAIN;

            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(lf), Math.abs(lb)),
                    Math.max(Math.abs(rf), Math.abs(rb))));
            lf /= max;
            lb /= max;
            rf /= max;
            rb /= max;

            leftFront.setPower(lf * speed);
            leftBack.setPower(lb * speed);
            rightFront.setPower(rf * speed);
            rightBack.setPower(rb * speed);

            telemetry.addData("Drive / Strafe / Turn", "%.2f / %.2f / %.2f", drive, strafe, turn);
            telemetry.addData("Slow mode", gamepad1.right_bumper ? "ON" : "off");
            telemetry.update();
        }
    }
}
