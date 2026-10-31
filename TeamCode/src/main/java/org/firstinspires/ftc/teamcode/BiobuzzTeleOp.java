package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * BIOBUZZ TeleOp: Strafer drive plus intake, launcher, Limelight auto-aim and match alerts.
 *
 * DRIVER (gamepad 1)
 *   Left stick            = drive / strafe
 *   Right stick X         = turn
 *   Right bumper (hold)   = slow mode (50%)
 *   Left bumper (hold)    = auto-aim at our HIVE's AprilTags + set launcher speed for the distance
 *   Back / Share          = field-centric ON/OFF (needs the Pinpoint)
 *   Start / Options       = field-centric reset: point the robot AWAY from you, then press
 *
 * OPERATOR (gamepad 2)
 *   Right trigger (hold)  = intake (stops by itself at 4 POLLEN - rule G407)
 *   A                     = spin up launcher       X = stop launcher
 *   Right bumper          = launch everything we hold (waits until the flywheel is at speed)
 *   Dpad up / down        = fix the POLLEN count if the sensor missed one
 *
 * RUMBLE ALERTS (both gamepads)
 *   2 blips  = intake full (4 POLLEN)
 *   long     = 60 s left: FLOWER scoring is now allowed (G410)
 *   3 blips  = 15 s left: head to the LOADING ZONE to park
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
    // Field-centric: +1 if the Pinpoint heading grows when the robot turns LEFT (counter-clockwise).
    // TODO: if field-centric drives the wrong way after turning the robot, set to -1.
    private static final double HEADING_SIGN = 1.0;
    // Auto-aim: turn power per degree of tx, capped so it can't spin wildly
    private static final double AIM_KP = 0.02;          // TODO: tune; flip the sign if it turns away
    private static final double AIM_MAX_TURN = 0.4;
    private static final double CAMERA_TX_OFFSET_DEG = 0; // TODO: same value as in BiobuzzAuto
    // Match timing (TELEOP is 2:00)
    private static final double TELEOP_SECONDS = 120;
    private static final double ENDGAME_LEFT = 60;   // FLOWER scoring opens (G410)
    private static final double PARK_WARNING_LEFT = 15;
    // ==================================================

    private DcMotor leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() {
        RobotUtil.enableBulkCaching(hardwareMap);

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
        // The Pinpoint keeps its heading from AUTO (we don't reset it here), so field-centric
        // works right away if the driver zeroes it once.
        GoBildaPinpointDriver pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, "pinpoint");
        // The auto ends with an empty robot (it launches everything), so start TeleOp at 0.
        intake.setCount(0);

        boolean red = BiobuzzAuto.lastAllianceRed;
        while (opModeInInit()) {
            if (gamepad1.x) red = true;
            if (gamepad1.b) red = false;
            double volts = RobotUtil.batteryVoltage(hardwareMap);
            telemetry.addData("Alliance (X red / B blue)", red ? "RED" : "BLUE");
            telemetry.addData("Battery", "%.2f V%s", volts, volts < RobotUtil.LOW_BATTERY_VOLTS ? "  LOW - SWAP IT" : "");
            telemetry.addData("Limelight", aligner.available() ? "found" : "not configured - no auto-aim");
            telemetry.addData("Pinpoint", pinpoint != null ? "found" : "not configured - robot-centric only");
            telemetry.addData("Last auto ended at", RobotUtil.lastAutoPose == null ? "none" : RobotUtil.lastAutoPose.toString());
            telemetry.update();
        }

        boolean lastUp = false, lastDown = false, lastBack = false;
        boolean firing = false, wasFull = false;
        boolean fieldCentric = false;
        double headingZero = 0;
        boolean endgameAlerted = false, parkAlerted = false;
        double volts = RobotUtil.batteryVoltage(hardwareMap);
        ElapsedTime matchTimer = new ElapsedTime();
        ElapsedTime voltTimer = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            // ---------------- DRIVE (gamepad 1)
            double stickFwd   = -gamepad1.left_stick_y;
            double stickRight =  gamepad1.left_stick_x;
            double turn       =  gamepad1.right_stick_x * TURN_DIRECTION;

            // Field-centric: "up on the stick" always means "away from the driver",
            // no matter which way the robot faces. Needs the robot's heading.
            if (pinpoint != null) {
                pinpoint.update();
                if (gamepad1.back && !lastBack) fieldCentric = !fieldCentric;
                if (gamepad1.start) headingZero = pinpoint.getHeading(AngleUnit.RADIANS);
            }
            lastBack = gamepad1.back;
            double drive = stickFwd, right = stickRight;
            if (fieldCentric && pinpoint != null) {
                double h = HEADING_SIGN * (pinpoint.getHeading(AngleUnit.RADIANS) - headingZero);
                // Rotate the stick direction from the field's frame into the robot's frame.
                drive = stickFwd * Math.cos(h) - stickRight * Math.sin(h);
                right = stickFwd * Math.sin(h) + stickRight * Math.cos(h);
            }
            double strafe = right * STRAFE_DIRECTION * STRAFE_BOOST;

            // Auto-aim replaces the driver's turn while held, if a tag of OUR HIVE is visible.
            // Either of our CELLs: red 30-37, blue 38-45. It also sets the launcher speed
            // for the distance (ShotTable), so the operator just presses fire.
            String aim = "off";
            if (gamepad1.left_bumper) {
                double[] t = red ? aligner.target(30, 37) : aligner.target(38, 45);
                if (t != null) {
                    turn = clamp(AIM_KP * (t[0] - CAMERA_TX_OFFSET_DEG), AIM_MAX_TURN);
                    if (!firing) launcher.setTargetRpm(ShotTable.rpmFor(t[1]));  // don't change speed mid-volley
                    aim = String.format("tx %.1f  ty %.1f", t[0], t[1]);
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

            // ---------------- ALERTS
            if (intake.isFull() && !wasFull) {
                gamepad1.rumbleBlips(2);
                gamepad2.rumbleBlips(2);
            }
            wasFull = intake.isFull();

            double left = TELEOP_SECONDS - matchTimer.seconds();
            if (!endgameAlerted && left <= ENDGAME_LEFT) {
                gamepad1.rumble(800);
                gamepad2.rumble(800);
                endgameAlerted = true;
            }
            if (!parkAlerted && left <= PARK_WARNING_LEFT) {
                gamepad1.rumbleBlips(3);
                gamepad2.rumbleBlips(3);
                parkAlerted = true;
            }

            // Voltage is a slow read, so only every half second.
            if (voltTimer.seconds() > 0.5) {
                volts = RobotUtil.batteryVoltage(hardwareMap);
                voltTimer.reset();
            }

            telemetry.addData("Time left", "%.0f s%s", Math.max(0, left),
                    left <= ENDGAME_LEFT ? "  ENDGAME: FLOWERS open" : "");
            telemetry.addData("Alliance", red ? "RED" : "BLUE");
            telemetry.addData("POLLEN held", "%d / %d%s", intake.count(), Intake.MAX_PIECES, intake.isFull() ? "  FULL" : "");
            telemetry.addData("Launcher", "%.0f / %.0f RPM %s", launcher.rpm(), launcher.targetRpm(), launcher.isReady() ? "READY" : "");
            telemetry.addData("Auto-aim", aim);
            telemetry.addData("Drive", "%s%s", fieldCentric ? "FIELD-centric" : "robot-centric",
                    gamepad1.right_bumper ? ", SLOW" : "");
            telemetry.addData("Battery / loop", "%.2f V / %.0f ms", volts, loopMs);
            telemetry.update();
        }
        aligner.stop();
    }

    private static double clamp(double v, double limit) {
        return Math.max(-limit, Math.min(limit, v));
    }
}
