package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/** Small helpers shared by every OpMode. */
public final class RobotUtil {

    private RobotUtil() {}

    /**
     * Where the robot was when AUTO ended. TeleOp reads this so it knows the robot's position
     * without re-measuring. Static fields survive between OpModes (until the app restarts).
     */
    public static Pose lastAutoPose = null;

    /**
     * Bulk reads: get every sensor/encoder value from a hub in ONE message per loop instead of
     * one message per read. This roughly halves loop time, which makes Pedro and every PID
     * react faster. AUTO mode refreshes the cache by itself, so nothing else has to change.
     */
    public static void enableBulkCaching(HardwareMap hw) {
        for (LynxModule hub : hw.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /** Lowest battery voltage any hub reports (0 if none). Reading it is slow: don't do it every loop. */
    public static double batteryVoltage(HardwareMap hw) {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor s : hw.voltageSensor) {
            double v = s.getVoltage();
            if (v > 0) min = Math.min(min, v);
        }
        return Double.isInfinite(min) ? 0 : min;
    }

    /** A battery under this sags during matches: swap it before queuing. */
    public static final double LOW_BATTERY_VOLTS = 12.5;
}
