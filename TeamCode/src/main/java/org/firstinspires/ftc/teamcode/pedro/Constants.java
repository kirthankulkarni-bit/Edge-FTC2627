package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Everything Pedro needs to know about our robot, in one place.
 *
 * A Follower is built from three parts:
 *   1. Drivetrain - which motors drive the robot (our goBILDA Strafer mecanum)
 *   2. Localizer  - how the robot knows where it is (goBILDA Pinpoint)
 *   3. Algorithm  - how Pedro steers along a path (Foresight)
 *
 * Every value marked TODO is a placeholder so the code compiles before the robot is ready.
 * Replace each one with the number the matching tuner prints (see teamcode/pedro/procedures).
 * Do NOT run a Pedro path on the real robot until the TODOs are filled in.
 */
public class Constants {

    // ================= DRIVETRAIN =================
    // Names and directions match the Driver Hub robot configuration exactly.
    // MecanumTuner confirms these on the real robot.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.backLeftName.set("leftBack");
        c.frontRightName.set("rightFront");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // ================= LOCALIZER (Pinpoint) =================
    // PinpointTuner measures the pod directions and offsets for us.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");  // TODO: must match the Pinpoint's name in the robot configuration
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);  // TODO: confirm pod type with build team
        c.xPodOffset.set(0.0);   // TODO: from PinpointTuner (or CAD), inches
        c.yPodOffset.set(0.0);   // TODO: from PinpointTuner (or CAD), inches
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);  // TODO: from PinpointTuner
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);  // TODO: from PinpointTuner
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // ================= PATH FOLLOWING (Foresight) =================
    // ForesightTuner prints this whole block. These placeholders are deliberately gentle
    // (low gains, low speeds) so an untuned robot moves slowly instead of wildly.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.forwardTranslational.set(Controller.proportional(0.05));  // TODO: ForesightTuner
        c.strafeTranslational.set(Controller.proportional(0.05));   // TODO: ForesightTuner
        c.headingFeedback.set(Controller.proportional(0.5));        // TODO: ForesightTuner
        c.coast.set(Controller.proportionalFeedforward(0.0));       // TODO: ForesightTuner
        c.brake.set(Controller.proportionalFeedforward(0.0));       // TODO: ForesightTuner

        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0, 0.0));     // TODO: ForesightTuner
        c.linearBrakeCoefficients.set(Matrix.diag(0.0, 0.0));             // TODO: ForesightTuner
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0, 0.0));          // TODO: ForesightTuner

        c.maxAchievableForwardVelocity.set(30.0);  // TODO: ForesightTuner, inches per second
        c.maxAchievableStrafeVelocity.set(25.0);   // TODO: ForesightTuner, inches per second
        c.naturalForwardDeceleration.set(30.0);    // TODO: ForesightTuner
        c.naturalStrafeDeceleration.set(30.0);     // TODO: ForesightTuner
    });

    /** Builds a ready-to-use Follower. Call once in init(). */
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
