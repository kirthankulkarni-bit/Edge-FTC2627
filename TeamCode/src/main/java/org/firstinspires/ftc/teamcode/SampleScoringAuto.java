package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Sample scoring auto built on ESTIMATED robot numbers. Adjust once the robot and CAD are done.
 *
 * Routine:
 *   1. Drive from the start to the goal while the lift rises, score the preloaded piece
 *   2. Lower the lift, drive to a pickup spot, pick up a piece
 *   3. Drive back to the goal while the lift rises, score again
 *   4. Lower the lift and curve into the park zone
 *
 * Every field position below is a PLACEHOLDER. Replace them with real spots from this season's
 * field (draw them in the Pedro visualizer). Coordinates: inches, (0,0) = field corner, heading in radians.
 *
 * The lift is simulated when no motor named "lift" is configured (see Lift.java),
 * and the scoring/intake actions are timed waits until those mechanisms exist.
 */
@Autonomous(name = "Sample Scoring Auto", group = "Practice", preselectTeleOp = "Strafer TeleOp")
public class SampleScoringAuto extends OpMode {

    // ================= TUNING SECTION =================
    // Field positions - PLACEHOLDERS (inches / radians). The robot is 18 in square, so its
    // center starts 9 in from the wall.
    static final Pose START  = new Pose(9, 63, Math.toRadians(0));
    static final Pose GOAL   = new Pose(30, 110, Math.toRadians(135));  // where we score
    static final Pose PICKUP = new Pose(40, 36, Math.toRadians(0));     // where we grab a piece
    static final Pose PARK_CONTROL = new Pose(60, 80);                  // bends the park curve
    static final Pose PARK   = new Pose(60, 36, Math.toRadians(90));

    static final double SCORE_SECONDS  = 0.5;   // TODO: time for the piece to leave the robot
    static final double INTAKE_SECONDS = 0.75;  // TODO: time to grab a piece
    static final double STEP_TIMEOUT_S = 5.0;   // safety: give up on any step after this
    static final double PARK_BY_SECONDS = 26.0; // skip remaining scoring and park if the clock passes this (auto is 30 s)
    // ==================================================

    private Follower follower;
    private Lift lift;
    private Path toGoal, toPickup, pickupToGoal, toPark;

    // Each state is one step. The loop checks "is this step done?" instead of waiting,
    // so follower.update() and lift.update() keep running and STOP always works.
    private enum State {
        DRIVE_TO_GOAL, SCORE_PRELOAD,
        DRIVE_TO_PICKUP, INTAKE,
        DRIVE_BACK_TO_GOAL, SCORE_SECOND,
        DRIVE_TO_PARK, DONE
    }
    private State state = State.DRIVE_TO_GOAL;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        lift = new Lift(hardwareMap);

        // .linear(a, b) turns smoothly from heading a to heading b while driving.
        toGoal = line(START, GOAL).linear(START.heading(), GOAL.heading());
        toPickup = line(GOAL, PICKUP).linear(GOAL.heading(), PICKUP.heading());
        pickupToGoal = line(PICKUP, GOAL).linear(PICKUP.heading(), GOAL.heading());
        // A curve: starts at GOAL, is pulled toward PARK_CONTROL, ends at PARK.
        toPark = curve(GOAL, PARK_CONTROL, PARK).linear(GOAL.heading(), PARK.heading());

        follower.setPose(START);
        telemetry.addData("Status", "Ready. Place robot at START.");
        telemetry.addData("Lift", lift.isSimulated() ? "SIMULATED (no 'lift' motor configured)" : "real motor");
    }

    @Override
    public void start() {
        follower.setPose(START);  // re-set in case the robot was nudged during init
        autoTimer.reset();
        follower.follow(toGoal);
        lift.setTarget(Lift.HIGH);  // raise WHILE driving, to save time
        stepTimer.reset();
    }

    @Override
    public void loop() {
        // Both MUST run every loop.
        follower.update();
        lift.update();

        boolean timedOut = stepTimer.seconds() > STEP_TIMEOUT_S;
        boolean outOfTime = autoTimer.seconds() > PARK_BY_SECONDS;
        boolean driveDone = !follower.isBusy();

        switch (state) {
            case DRIVE_TO_GOAL:
                // Wait for BOTH the drive and the lift before scoring.
                if ((driveDone && lift.atTarget()) || timedOut) {
                    // TODO: open the claw / run the outtake here
                    next(State.SCORE_PRELOAD);
                }
                break;

            case SCORE_PRELOAD:
                if (stepTimer.seconds() > SCORE_SECONDS) {
                    lift.setTarget(Lift.DOWN);
                    if (outOfTime) {
                        goPark();
                    } else {
                        follower.follow(toPickup);
                        next(State.DRIVE_TO_PICKUP);
                    }
                }
                break;

            case DRIVE_TO_PICKUP:
                if (driveDone || timedOut) {
                    // TODO: run the intake here
                    next(State.INTAKE);
                }
                break;

            case INTAKE:
                if (stepTimer.seconds() > INTAKE_SECONDS) {
                    // TODO: stop the intake / close the claw here
                    if (outOfTime) {
                        goPark();
                    } else {
                        follower.follow(pickupToGoal);
                        lift.setTarget(Lift.HIGH);
                        next(State.DRIVE_BACK_TO_GOAL);
                    }
                }
                break;

            case DRIVE_BACK_TO_GOAL:
                if ((driveDone && lift.atTarget()) || timedOut) {
                    // TODO: open the claw / run the outtake here
                    next(State.SCORE_SECOND);
                }
                break;

            case SCORE_SECOND:
                if (stepTimer.seconds() > SCORE_SECONDS) {
                    lift.setTarget(Lift.DOWN);
                    goPark();
                }
                break;

            case DRIVE_TO_PARK:
                if (driveDone || timedOut) {
                    follower.stop();
                    next(State.DONE);
                }
                break;

            case DONE:
                break;  // keep updating the lift so it stays down; wait for STOP
        }

        Pose p = follower.pose();
        telemetry.addData("State", state);
        telemetry.addData("Auto time", "%.1f s", autoTimer.seconds());
        telemetry.addData("x / y / heading deg", "%.1f / %.1f / %.0f", p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.addData("Lift target / pos", "%d / %d%s", lift.target(), lift.position(),
                lift.isSimulated() ? " (sim)" : "");
    }

    /** Parking uses the curve from the goal. If we're not at the goal, Pedro still drives the curve from where it is. */
    private void goPark() {
        follower.follow(toPark);
        next(State.DRIVE_TO_PARK);
    }

    private void next(State s) {
        state = s;
        stepTimer.reset();
    }
}
