package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Pedro version of BasicAuto: forward 12 in, turn 180 degrees, forward 12 in.
 *
 * Do not run on the robot until pedro/Constants.java has real tuned values (no TODOs left).
 */
@Autonomous(name = "Pedro Auto", group = "Practice", preselectTeleOp = "Strafer TeleOp")
public class PedroAuto extends OpMode {

    // ================= TUNING SECTION =================
    static final double DRIVE_INCHES = 12.0;
    static final double TURN_TOLERANCE_RAD = Math.toRadians(3); // "close enough" for the turn
    static final double STEP_TIMEOUT_S = 4.0;                    // safety: give up on a step after this
    // ==================================================

    // Start in the middle of the field facing +x. Units: inches and radians.
    private final Pose start  = new Pose(72, 72, 0);
    private final Pose out    = new Pose(72 + DRIVE_INCHES, 72, 0);
    private final Pose turned = out.withHeading(Math.PI);
    private final Pose back   = new Pose(72, 72, Math.PI);

    private Follower follower;
    private Path forward1, forward2;

    // Each state is one step. The loop checks "is this step done?" instead of waiting,
    // so follower.update() keeps running and the STOP button always works.
    private enum State { DRIVE_OUT, TURN, DRIVE_BACK, DONE }
    private State state = State.DRIVE_OUT;
    private final ElapsedTime stepTimer = new ElapsedTime();

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        // .constant(h) keeps the robot facing h the whole way, so it drives straight.
        forward1 = line(start, out).constant(0);
        forward2 = line(turned, back).constant(Math.PI);
        follower.setPose(start);
        telemetry.addData("Status", "Ready. Place robot at the start pose.");
    }

    @Override
    public void start() {
        follower.setPose(start);  // re-set in case the robot was nudged during init
        follower.follow(forward1);
        stepTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();  // MUST run every loop: reads Pinpoint, drives motors

        boolean timedOut = stepTimer.seconds() > STEP_TIMEOUT_S;
        switch (state) {
            case DRIVE_OUT:
                if (!follower.isBusy() || timedOut) {
                    follower.hold(turned);  // hold this spot but face 180 degrees: a turn in place
                    next(State.TURN);
                }
                break;
            case TURN:
                double err = Math.abs(Angle.smallestDifference(follower.pose().heading(), Math.PI));
                if (err < TURN_TOLERANCE_RAD || timedOut) {
                    follower.follow(forward2);
                    next(State.DRIVE_BACK);
                }
                break;
            case DRIVE_BACK:
                if (!follower.isBusy() || timedOut) {
                    follower.stop();
                    next(State.DONE);
                }
                break;
            case DONE:
                break;  // sit still until STOP is pressed
        }

        Pose p = follower.pose();
        telemetry.addData("State", state);
        telemetry.addData("x / y / heading deg", "%.1f / %.1f / %.0f", p.x(), p.y(), Math.toDegrees(p.heading()));
    }

    private void next(State s) {
        state = s;
        stepTimer.reset();
    }
}
