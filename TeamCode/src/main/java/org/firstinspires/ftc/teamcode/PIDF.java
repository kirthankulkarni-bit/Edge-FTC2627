package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A basic PIDF controller. Call calculate() once per loop.
 * Units are whatever you measure in (encoder ticks, inches, degrees) - just be consistent.
 */
public class PIDF {
    public double kP, kI, kD, kF;

    // Caps the I term so error built up while stuck (e.g. lift blocked) can't explode.
    public double maxIntegral = 0.25;

    private double integral = 0;
    private double lastError = 0;
    private boolean first = true;
    private final ElapsedTime timer = new ElapsedTime();

    public PIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /** Returns motor power to move `current` toward `target`. */
    public double calculate(double target, double current) {
        double error = target - current;
        double dt = timer.seconds();
        timer.reset();

        // First call has no previous error or time step, so skip I and D once.
        if (first || dt <= 0) {
            first = false;
            lastError = error;
            return kP * error + kF;
        }

        integral += error * dt;
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
        double derivative = (error - lastError) / dt;
        lastError = error;

        return kP * error + kI * integral + kD * derivative + kF;
    }

    /** Call when the target changes a lot, so old I and D history doesn't cause a jump. */
    public void reset() {
        integral = 0;
        first = true;
        timer.reset();
    }
}
