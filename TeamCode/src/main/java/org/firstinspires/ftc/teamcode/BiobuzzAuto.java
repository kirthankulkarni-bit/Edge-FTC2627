package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * BIOBUZZ (2026-27) autonomous. Shared by BiobuzzAutoRed and BiobuzzAutoBlue.
 *
 * FULL route (red side, matches paths/biobuzz-red-auto.pp in the Pedro visualizer):
 *   1. Pull off the audience wall (LEAVE, 3 pts), align on our CELL's AprilTags, launch the
 *      4 preloads into our raised CELL. With the 3 pre-staged NECTAR this should TIP (20 pts).
 *   2. Drive to the Red Alliance FLOWER and intake until the sensor counts 4 POLLEN.
 *   3. Go up the side corridor, then around the HIVE to the far side (our other CELL is raised
 *      after the tip), align, launch.
 *   4. Park in our LOADING ZONE (5 pts).
 *
 * BEFORE THE MATCH (during INIT, on gamepad 1):
 *   dpad up / down    = start delay +/- 0.5 s (to stay out of an alliance partner's way)
 *   dpad left / right = choose the route (see Route below)
 *
 * Coordinates: Pedro visualizer BIOBUZZ field. Inches, (0,0) = audience-wall corner on the red
 * side, x toward blue, y toward the far wall. Blue = red route rotated 180 degrees.
 */
public abstract class BiobuzzAuto extends OpMode {

    // ================= TUNING SECTION =================
    // Red-side poses {x, y, heading deg}. Blue is computed by mirror() in pose().
    static final double[] START      = {56, 9, 90};    // touching audience wall, facing the HIVE
    static final double[] SHOOT_NEAR = {56, 22, 90};   // ~28 in from the HIVE base, raised CELL ahead
    static final double[] FLOWER     = {16, 47, 180};  // facing the Red Alliance FLOWER (wall, y 43.7-50.4)
    // Far-side trip, split in two (see buildPaths() for why):
    static final double[] CORRIDOR   = {24, 100, 270}; // end of the straight run up the side corridor
    static final double[] FAR_CTRL   = {28, 124};      // control point of the short final arc
    static final double[] SHOOT_FAR  = {60, 126, 270}; // facing the far-side CELL
    static final double[] SIDE_LANE  = {18, 50, 180};  // waypoint that keeps park-only routes off the HIVE
    static final double[] PARK       = {12, 108, 180}; // overlaps LOADING ZONE (x 0-11, y 96-120)

    // Timing
    static final double SHOOT_SECONDS   = 2.5;  // TODO: spin-up + 4 launches, from the real launcher
    static final double INTAKE_TIMEOUT  = 2.5;  // leave the FLOWER even if we have fewer than 4
    static final double ALIGN_TIMEOUT   = 0.8;  // never spend longer than this aligning
    static final double STEP_TIMEOUT_S  = 5.0;  // safety: give up on any drive after this
    static final double PARK_BY_SECONDS = 25.0; // skip to parking after this (AUTO is 30 s)
    static final double MAX_START_DELAY = 10.0;

    // AprilTag alignment
    static final double ALIGN_TOLERANCE_DEG = 1.5; // close enough to shoot
    static final int    ALIGN_STABLE_LOOPS  = 3;   // must be inside tolerance this many loops in a row
    static final double ALIGN_GAIN          = 0.8; // fraction of tx corrected per update (<1 avoids overshoot)
    static final double CAMERA_TX_OFFSET_DEG = 0;  // TODO: tx reading when the launcher is truly on target
    // ==================================================

    /** Alliance-partner deconfliction: pick in INIT with dpad left/right. */
    enum Route {
        FULL,          // preloads near, FLOWER, far CELL, park (best when the hive tips)
        NEAR_TWICE,    // preloads near, FLOWER, back to the near CELL, park (hive didn't tip / partner uses far side)
        PRELOAD_PARK,  // preloads near, then park (partner needs our FLOWER lane)
        PARK_ONLY      // LEAVE + PARK only, 8 pts (launcher broken)
    }

    /** true for the red OpMode, false for blue. */
    protected abstract boolean isRed();

    private enum State {
        START_DELAY, TO_SHOOT_1, ALIGN_1, SHOOT_1,
        TO_FLOWER, INTAKE, TO_SHOOT_2, ALIGN_2, SHOOT_2,
        TO_PARK, DONE
    }

    private Follower follower;
    private Intake intake;
    private TagAligner aligner;

    private Route route = Route.FULL;
    private double startDelay = 0;
    private boolean lastUp, lastDown, lastLeft, lastRight;

    private Pose start, shootNear, flower, shootFar, park;
    private Path toShoot1, toFlower, toShoot2, toPark, parkFromStart;
    private Pose shot2;        // where the second batch is launched from
    private int[] cell1, cell2; // tag ID ranges {min, max} for each shot

    private State state = State.START_DELAY;
    private Pose alignTarget;
    private int stableLoops;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    // ---------------------------------------------------------------- INIT

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        intake = new Intake(hardwareMap);
        aligner = new TagAligner(hardwareMap);
        intake.setCount(4);  // G304: exactly 4 preloads
    }

    /** Runs repeatedly between INIT and START: lets the drive team set the delay and route. */
    @Override
    public void init_loop() {
        if (gamepad1.dpad_up && !lastUp) startDelay = Math.min(MAX_START_DELAY, startDelay + 0.5);
        if (gamepad1.dpad_down && !lastDown) startDelay = Math.max(0, startDelay - 0.5);
        Route[] all = Route.values();
        if (gamepad1.dpad_right && !lastRight) route = all[(route.ordinal() + 1) % all.length];
        if (gamepad1.dpad_left && !lastLeft) route = all[(route.ordinal() + all.length - 1) % all.length];
        // Edge detection: one press = one step, no matter how long the button is held.
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;

        telemetry.addData("Alliance", isRed() ? "RED" : "BLUE");
        telemetry.addData("Route (dpad L/R)", route);
        telemetry.addData("Start delay (dpad U/D)", "%.1f s", startDelay);
        telemetry.addData("Limelight", aligner.available() ? "found" : "NOT configured - no aligning");
        telemetry.addData("Intake count", intake.countSource());
    }

    @Override
    public void start() {
        buildPaths();  // built here so the chosen route is used
        follower.setPose(start);
        autoTimer.reset();
        next(State.START_DELAY);
    }

    // ---------------------------------------------------------------- PATHS

    private void buildPaths() {
        start = pose(START);
        shootNear = pose(SHOOT_NEAR);
        flower = pose(FLOWER);
        shootFar = pose(SHOOT_FAR);
        park = pose(PARK);
        Pose corridor = pose(CORRIDOR);
        Pose farCtrl = pose(new double[]{FAR_CTRL[0], FAR_CTRL[1], 0});
        Pose sideLane = pose(SIDE_LANE);

        // Tag clusters (Competition Manual 9.9). Red's first shot is the audience-side CELL;
        // blue's route is rotated, so its first shot is at blue's far-side CELL.
        int[] redNear = {34, 37}, redFar = {30, 33}, blueFar = {42, 45}, blueNear = {38, 41};
        cell1 = isRed() ? redNear : blueFar;

        // Short straight move off the wall. Constant heading: we only need to clear the wall.
        toShoot1 = line(start, shootNear).constant(shootNear.heading());

        // Straight line, turning while it drives: the turn is spread over ~47 in of travel.
        toFlower = line(shootNear, flower).linear(shootNear.heading(), flower.heading());

        if (route == Route.FULL) {
            shot2 = shootFar;
            cell2 = isRed() ? redFar : blueNear;
            // DRIFT FIX. The old version was one long arc FLOWER -> (24,112) -> SHOOT_FAR that also
            // turned 90 degrees along the way. Turning and arcing at speed is when mecanum wheels slip
            // on foam and the robot swings wide (toward the HIVE). Now it is two pieces:
            //   a) a STRAIGHT run up the open side corridor (x 16-24, far from the HIVE at x>=46).
            //      The 90-degree turn happens here, where a little slip can't cause a collision.
            //   b) a SHORT, GENTLE arc around the far end of the HIVE with the heading held CONSTANT,
            //      so all the wheel grip goes to the curve. FAR_CTRL sits only 4 in right of the
            //      corridor line, which keeps the curvature low (low curvature = low sideways force).
            // The two pieces meet at CORRIDOR heading almost the same way (line direction 8:53,
            // arc's starting direction 4:24 - both ~9 degrees right of straight up), so there is
            // no sudden change of direction at the joint.
            toShoot2 = path(
                    line(flower, corridor).linear(flower.heading(), corridor.heading()),
                    curve(corridor, farCtrl, shootFar).constant(shootFar.heading()));
        } else {
            shot2 = shootNear;
            cell2 = cell1;
            toShoot2 = line(flower, shootNear).linear(flower.heading(), shootNear.heading());
        }

        // Park routes. From the far shot the run to the LOADING ZONE is clear of the HIVE.
        // From the near side, a straight line would clip the HIVE's corner, so go via SIDE_LANE.
        if (route == Route.FULL) {
            toPark = line(shot2, park).linear(shot2.heading(), park.heading());
        } else {
            toPark = path(
                    line(shot2, sideLane).linear(shot2.heading(), sideLane.heading()),
                    line(sideLane, park).constant(park.heading()));
        }
        parkFromStart = path(
                line(start, sideLane).linear(start.heading(), sideLane.heading()),
                line(sideLane, park).constant(park.heading()));
    }

    // ---------------------------------------------------------------- LOOP

    @Override
    public void loop() {
        // These MUST run every loop. "Pausing" a path never means skipping update():
        // we pause by switching the follower to hold() a pose instead.
        follower.update();
        intake.update();

        boolean driveDone = !follower.isBusy();
        boolean timedOut = stepTimer.seconds() > STEP_TIMEOUT_S;

        // Parking is worth 5 pts: never let a slow step cost us the park.
        if (autoTimer.seconds() > PARK_BY_SECONDS && state != State.TO_PARK && state != State.DONE) {
            intake.stop();
            Pose here = follower.pose();
            follower.follow(line(here, park).linear(here.heading(), park.heading()));
            next(State.TO_PARK);
        }

        switch (state) {
            case START_DELAY:
                // Timer-based wait (no sleep()), so STOP still works and Pedro keeps holding still.
                if (stepTimer.seconds() >= startDelay) {
                    if (route == Route.PARK_ONLY) {
                        follower.follow(parkFromStart);
                        next(State.TO_PARK);
                    } else {
                        follower.follow(toShoot1);
                        // TODO: start spinning up the launcher here so it's ready on arrival
                        next(State.TO_SHOOT_1);
                    }
                }
                break;

            case TO_SHOOT_1:
                if (driveDone || timedOut) beginAlign(shootNear, State.ALIGN_1);
                break;
            case ALIGN_1:
                if (aligned(cell1)) next(State.SHOOT_1);  // TODO: start feeding the launcher here
                break;
            case SHOOT_1:
                if (stepTimer.seconds() > SHOOT_SECONDS) {
                    intake.setCount(0);  // TODO: count down per launch if the launcher has a sensor
                    if (route == Route.PRELOAD_PARK) {
                        follower.follow(toPark);
                        next(State.TO_PARK);
                    } else {
                        follower.follow(toFlower);
                        intake.start();  // spin up on the way so pieces are grabbed on contact
                        next(State.TO_FLOWER);
                    }
                }
                break;

            case TO_FLOWER:
                // The sensor may already count pieces on arrival; leave as soon as we have 4.
                if (intake.isFull()) {
                    leaveFlower();
                } else if (driveDone || timedOut) {
                    next(State.INTAKE);
                }
                break;
            case INTAKE:
                // Leave the moment the 4th piece is counted (the Intake class stops itself),
                // or after INTAKE_TIMEOUT with whatever we have - a partial batch still scores.
                if (intake.isFull() || stepTimer.seconds() > INTAKE_TIMEOUT) leaveFlower();
                break;

            case TO_SHOOT_2:
                if (driveDone || timedOut) beginAlign(shot2, State.ALIGN_2);
                break;
            case ALIGN_2:
                if (aligned(cell2)) next(State.SHOOT_2);  // TODO: start feeding the launcher here
                break;
            case SHOOT_2:
                if (stepTimer.seconds() > SHOOT_SECONDS) {
                    intake.setCount(0);
                    follower.follow(toPark);
                    next(State.TO_PARK);
                }
                break;

            case TO_PARK:
                if (driveDone || timedOut) {
                    follower.stop();
                    aligner.stop();
                    next(State.DONE);
                }
                break;
            case DONE:
                break;
        }

        Pose p = follower.pose();
        telemetry.addData("Alliance / route", "%s / %s", isRed() ? "RED" : "BLUE", route);
        telemetry.addData("State", state);
        telemetry.addData("Auto time", "%.1f s", autoTimer.seconds());
        telemetry.addData("x / y / heading deg", "%.1f / %.1f / %.0f", p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.addData("POLLEN held", "%d (%s)", intake.count(), intake.countSource());
    }

    // ---------------------------------------------------------------- HELPERS

    private void leaveFlower() {
        intake.stop();
        follower.follow(toShoot2);
        // TODO: spin up the launcher here
        next(State.TO_SHOOT_2);
    }

    /** The path ended: stop following it and HOLD the shooting pose while vision fine-tunes the aim. */
    private void beginAlign(Pose shotPose, State alignState) {
        alignTarget = shotPose;
        follower.hold(alignTarget);  // Pedro keeps correcting position; we only adjust heading
        stableLoops = 0;
        next(alignState);
    }

    /**
     * One step of AprilTag alignment. Returns true when it's time to shoot.
     * Uses tx (the tag's left/right angle) only, so no field-coordinate conversion is needed.
     */
    private boolean aligned(int[] cell) {
        if (!aligner.available()) return true;               // no camera: shoot at the planned pose
        if (stepTimer.seconds() > ALIGN_TIMEOUT) return true; // never let vision cost us the shot

        double tx = aligner.txTo(cell[0], cell[1]);
        if (Double.isNaN(tx)) return false;  // tag not seen yet: keep holding, wait (until timeout)

        double error = tx - CAMERA_TX_OFFSET_DEG;
        if (Math.abs(error) < ALIGN_TOLERANCE_DEG) {
            return ++stableLoops >= ALIGN_STABLE_LOOPS;  // must stay on target a few loops
        }
        stableLoops = 0;
        // Tag to the right (positive tx) means turn clockwise, which is a SMALLER Pedro heading.
        double newHeading = follower.pose().heading() - Math.toRadians(error) * ALIGN_GAIN;
        alignTarget = alignTarget.withHeading(newHeading);
        follower.hold(alignTarget);
        return false;
    }

    /** Builds a Pose from a red {x, y, headingDeg}, rotated 180 degrees about the center for blue. */
    private Pose pose(double[] red) {
        double h = Math.toRadians(red[2]);
        if (isRed()) return new Pose(red[0], red[1], h);
        return new Pose(144 - red[0], 144 - red[1], h + Math.PI);
    }

    private void next(State s) {
        state = s;
        stepTimer.reset();
    }
}
