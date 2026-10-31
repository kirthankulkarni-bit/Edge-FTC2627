package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Intake that counts POLLEN and never lets the robot hold more than 4 (rule G407).
 *
 * Counting uses a beam-break sensor across the intake entrance: each time the beam goes from
 * clear to broken, one POLLEN has entered. As soon as the count reaches MAX_PIECES the intake
 * runs backwards for a moment (to push out a 5th piece already touching the rollers) and stops.
 *
 * Missing hardware is handled so the auto still runs before the mechanism is built:
 *   no "intake" motor      -> nothing spins
 *   no "intakeBeam" sensor -> pieces are counted by time (SIM_SECONDS_PER_PIECE)
 * Telemetry from countSource() says which one is in use.
 *
 * Other sensors work the same way: a distance sensor ("piece present when < X cm") or a color
 * sensor in each slot. Swap the detection lines in update() for whatever the build team mounts.
 */
public class Intake {

    // ================= TUNING SECTION =================
    public static int MAX_PIECES = 4;                // G407: never CONTROL more than 4
    public static double INTAKE_POWER = 1.0;
    public static double EJECT_POWER = -0.6;         // brief reverse after the 4th piece
    public static double EJECT_SECONDS = 0.15;
    public static double SIM_SECONDS_PER_PIECE = 0.35; // used only when no beam-break is configured
    // ==================================================

    private enum Mode { OFF, INTAKING, EJECTING }

    private final DcMotor motor;          // null if not configured
    private final DigitalChannel beam;    // null if not configured
    private Mode mode = Mode.OFF;
    private int count = 0;
    private boolean lastBroken = false;
    private final ElapsedTime timer = new ElapsedTime();

    public Intake(HardwareMap hw) {
        motor = hw.tryGet(DcMotor.class, "intake");            // TODO: match robot configuration
        beam = hw.tryGet(DigitalChannel.class, "intakeBeam");  // TODO: match robot configuration
        if (motor != null) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (beam != null) beam.setMode(DigitalChannel.Mode.INPUT);
    }

    /** Start pulling POLLEN in. Does nothing if already full. */
    public void start() {
        if (isFull()) return;
        mode = Mode.INTAKING;
        timer.reset();
    }

    public void stop() { mode = Mode.OFF; }

    /** Call every loop. */
    public void update() {
        switch (mode) {
            case INTAKING:
                setPower(INTAKE_POWER);
                if (pieceEntered()) count++;
                if (isFull()) {
                    // Full: reverse briefly so we can't end up controlling a 5th piece.
                    mode = Mode.EJECTING;
                    timer.reset();
                }
                break;
            case EJECTING:
                setPower(EJECT_POWER);
                if (timer.seconds() > EJECT_SECONDS) mode = Mode.OFF;
                break;
            case OFF:
                setPower(0);
                break;
        }
    }

    /** True once on each new piece. */
    private boolean pieceEntered() {
        if (beam != null) {
            // REV-style beam-break sensors read LOW (false) when the beam is broken.
            boolean broken = !beam.getState();
            boolean entered = broken && !lastBroken;  // count the clear -> broken edge only
            lastBroken = broken;
            return entered;
        }
        // Simulated: one piece every SIM_SECONDS_PER_PIECE.
        if (timer.seconds() > SIM_SECONDS_PER_PIECE) {
            timer.reset();
            return true;
        }
        return false;
    }

    private void setPower(double p) {
        if (motor != null) motor.setPower(p);
    }

    public boolean isFull() { return count >= MAX_PIECES; }
    public boolean isRunning() { return mode != Mode.OFF; }
    public int count() { return count; }
    /** Set after launching (0) or at the start of auto (preloads). */
    public void setCount(int n) { count = n; }
    public String countSource() { return beam != null ? "beam-break" : "SIMULATED (no intakeBeam)"; }
}
