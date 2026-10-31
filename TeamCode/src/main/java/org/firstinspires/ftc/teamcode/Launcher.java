package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Flywheel launcher with a feeder that pushes POLLEN into it one at a time.
 *
 * Assumed design (TODO: change to match what the build team builds):
 *   "launcher" - flywheel motor, speed-controlled with its encoder
 *   "feeder"   - motor that pushes the next POLLEN into the flywheel
 *
 * Usage: spinUp() early (on the drive to the shooting spot), then fire(n) once aimed.
 * fire() feeds only while the flywheel is at speed, one piece every SHOT_INTERVAL seconds,
 * so each shot leaves at the same speed (consistent shots = consistent HIVE TIPs).
 *
 * If "launcher" isn't configured, everything is SIMULATED with the same timing, so autos run now.
 */
public class Launcher {

    // ================= TUNING SECTION =================
    public static double TARGET_RPM = 3500;      // TODO: tune for the distance to the CELL
    public static double TICKS_PER_REV = 28;     // goBILDA 5203 1:1 motor (6000 RPM). TODO: confirm
    public static double READY_TOLERANCE_RPM = 150; // "at speed" if within this
    public static double FEED_POWER = 1.0;
    public static double SHOT_INTERVAL = 0.35;   // seconds between pieces (lets the flywheel recover)
    public static double SIM_SPINUP_SECONDS = 1.0;
    // ==================================================

    private final DcMotorEx flywheel;  // null when simulated
    private final DcMotor feeder;      // null if not configured
    private boolean spinning = false;
    private int shotsLeft = 0;
    private final ElapsedTime spinTimer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();

    public Launcher(HardwareMap hw) {
        flywheel = hw.tryGet(DcMotorEx.class, "launcher");  // TODO: match robot configuration
        feeder = hw.tryGet(DcMotor.class, "feeder");          // TODO: match robot configuration
        if (flywheel != null) {
            flywheel.setDirection(DcMotorSimple.Direction.FORWARD);  // TODO: must launch outward
            // RUN_USING_ENCODER lets setVelocity() hold a steady speed even as the battery drops.
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // coast, don't slam to a stop
        }
    }

    public void spinUp() {
        if (!spinning) spinTimer.reset();
        spinning = true;
    }

    /** Stops the flywheel and cancels any shots still waiting. */
    public void stop() {
        spinning = false;
        shotsLeft = 0;
    }

    /** Launch n pieces (spins up first if needed). */
    public void fire(int n) {
        spinUp();
        shotsLeft = Math.max(0, n);
        shotTimer.reset();
    }

    /** Call every loop. */
    public void update() {
        if (flywheel != null) {
            flywheel.setVelocity(spinning ? TARGET_RPM / 60.0 * TICKS_PER_REV : 0);
        }
        boolean feeding = shotsLeft > 0 && isReady();
        if (feeder != null) feeder.setPower(feeding ? FEED_POWER : 0);
        // One piece leaves every SHOT_INTERVAL while feeding.
        // TODO: if the build team adds a sensor at the flywheel, count real shots instead.
        if (feeding && shotTimer.seconds() > SHOT_INTERVAL) {
            shotsLeft--;
            shotTimer.reset();
        } else if (!feeding) {
            shotTimer.reset();
        }
    }

    public boolean isReady() {
        if (!spinning) return false;
        if (flywheel == null) return spinTimer.seconds() > SIM_SPINUP_SECONDS;
        return Math.abs(rpm() - TARGET_RPM) < READY_TOLERANCE_RPM;
    }

    /** True when every requested piece has been launched. */
    public boolean isDone() { return shotsLeft == 0; }

    public int shotsLeft() { return shotsLeft; }

    public double rpm() {
        return flywheel == null ? (isReady() ? TARGET_RPM : 0) : flywheel.getVelocity() / TICKS_PER_REV * 60.0;
    }

    public boolean isSimulated() { return flywheel == null; }
}
