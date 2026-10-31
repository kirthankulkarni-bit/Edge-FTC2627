package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Linear-slide lift held at a target height by a PIDF loop.
 *
 * If there is no motor named "lift" in the robot configuration (the lift isn't built yet),
 * this class SIMULATES one: the position moves toward the target at SIM_TICKS_PER_SECOND.
 * That lets autos that use the lift run today. Telemetry shows "SIMULATED" when this happens.
 */
public class Lift {

    // ================= TUNING SECTION =================
    public static double KP = 0.005;   // TODO: tune (power per tick of error)
    public static double KI = 0.0;     // usually leave at 0
    public static double KD = 0.0002;  // TODO: tune
    public static double KF = 0.08;    // TODO: power that just holds the lift still
    public static int TOLERANCE_TICKS = 20;  // "close enough" to count as arrived
    public static double MAX_POWER = 1.0;

    // Target heights in encoder ticks. TODO: measure on the real lift.
    public static final int DOWN = 0;
    public static final int LOW = 800;
    public static final int HIGH = 1600;

    // Simulated lift speed. A 312 RPM motor (537.7 ticks/rev) turns ~2800 ticks/s with no load;
    // under the weight of a lift, assume about 70% of that.
    public static double SIM_TICKS_PER_SECOND = 2000;
    // ==================================================

    private final DcMotorEx motor;  // null when simulated
    private final PIDF pid = new PIDF(KP, KI, KD, KF);
    private int target = DOWN;

    private double simPosition = DOWN;
    private final ElapsedTime simTimer = new ElapsedTime();

    public Lift(HardwareMap hw) {
        // tryGet returns null instead of crashing when the device isn't configured.
        motor = hw.tryGet(DcMotorEx.class, "lift");  // TODO: match robot configuration name
        if (motor != null) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);  // TODO: up must = positive ticks
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Reset so "0" means "fully down" at the start of auto.
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // WITHOUT_ENCODER: our PID sets the power. The encoder still counts.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setTarget(int ticks) {
        if (ticks != target) pid.reset();
        target = ticks;
    }

    /** Call every loop, just like follower.update(). */
    public void update() {
        if (motor == null) {
            // Simulated: move toward the target at a fixed speed.
            double step = SIM_TICKS_PER_SECOND * simTimer.seconds();
            simTimer.reset();
            double error = target - simPosition;
            simPosition += Math.signum(error) * Math.min(Math.abs(error), step);
            return;
        }
        double power = pid.calculate(target, motor.getCurrentPosition());
        motor.setPower(Math.max(-MAX_POWER, Math.min(MAX_POWER, power)));
    }

    public boolean atTarget() {
        return Math.abs(target - position()) < TOLERANCE_TICKS;
    }

    public int position() {
        return motor == null ? (int) Math.round(simPosition) : motor.getCurrentPosition();
    }

    public int target() { return target; }

    public boolean isSimulated() { return motor == null; }
}
