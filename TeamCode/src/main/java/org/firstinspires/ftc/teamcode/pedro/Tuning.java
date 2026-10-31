package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * Registers the Pedro tuners we use. Each @Tuner method is found automatically at startup.
 *
 * To run them: connect the laptop to the robot's Wi-Fi and open http://192.168.43.1:10158
 * (the tuning web page). Pick a tuner there and follow its instructions.
 *
 * Run them in this order - each one's printed config goes into Constants.java before the next:
 *   1. Mecanum Tuner  -> drivetrainConfig (motor names and directions)
 *   2. Pinpoint Tuner -> localizerConfig  (pod directions and offsets)
 *   3. Tests: Localization / Odometry -> check the robot knows where it is
 *   4. Foresight Tuner -> foresightConfig (all the TODO path-following numbers)
 *   5. Tests: Line / Curve / Hold -> check it drives paths cleanly
 */
public class Tuning {

    @Tuner(name = "1 Mecanum Tuner")
    static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "2 Pinpoint Tuner")
    static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner(name = "3 Foresight Tuner")
    static Procedure foresight() {
        return new ForesightTuner(
                h -> new PinpointLocalizer(h, Constants.localizerConfig),
                h -> new Mecanum(h, Constants.drivetrainConfig));
    }

    @Tuner(name = "Tests")
    static Procedure tests() {
        return new Tests(
                h -> new Mecanum(h, Constants.drivetrainConfig),
                h -> new PinpointLocalizer(h, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig));
    }
}
