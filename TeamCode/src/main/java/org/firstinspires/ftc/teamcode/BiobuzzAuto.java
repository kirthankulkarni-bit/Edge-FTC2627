package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * BIOBUZZ (2026-27) autonomous. Shared by BiobuzzAutoRed and BiobuzzAutoBlue.
 *
 * Plan (red side, matches paths/biobuzz-red-auto.pp in the Pedro visualizer):
 *   1. Pull off the audience wall (LEAVE, 3 pts) and launch the 4 preloads into our raised
 *      (audience-side) CELL. With the 3 pre-staged NECTAR this should TIP the HIVE (20 pts).
 *   2. Drive to our Red Alliance FLOWER on the side wall and intake its 4 POLLEN.
 *   3. Curve around the HIVE to the far side, where our other CELL is now raised, and launch them.
 *   4. Park in our LOADING ZONE (5 pts).
 * Estimated 15.4 s with our estimated speeds, so there is ~14 s of margin in the 30 s AUTO.
 *
 * Coordinates: Pedro visualizer BIOBUZZ field. Inches, (0,0) = corner at the audience wall on
 * the red side, x toward the blue side, y toward the far (scoring-side) wall. Heading in degrees
 * below, converted to radians. Blue is the red route rotated 180 degrees about the field center.
 */
public abstract class BiobuzzAuto extends OpMode {

    // ================= TUNING SECTION =================
    // Red-side poses (x, y, heading deg). Blue is computed by mirror().
    static final double[] START       = {56, 9, 90};    // touching audience wall, facing the HIVE
    static final double[] SHOOT_NEAR  = {56, 22, 90};   // ~28 in from the HIVE base, raised CELL ahead
    static final double[] FLOWER      = {16, 47, 180};  // facing the Red Alliance FLOWER (left wall, y 43.7-50.4)
    static final double[] FAR_CONTROL = {24, 112};      // pulls the curve around the HIVE (HIVE x 46-96.5, y 50-91.4)
    static final double[] SHOOT_FAR   = {60, 126, 270}; // facing the far-side CELL
    static final double[] PARK        = {12, 108, 180}; // overlaps LOADING ZONE (x 0-11, y 96-120)

    // TODO: set after testing whether the 4 preloads really tip the HIVE (sources say 3-4 POLLEN).
    // false = the HIVE did not tip, so the second batch goes into the same near CELL again.
    static final boolean SECOND_BATCH_FAR_SIDE = true;

    static final double SHOOT_SECONDS  = 2.5;  // TODO: spin-up + 4 launches, from the real launcher
    static final double INTAKE_SECONDS = 1.5;  // TODO: time to pull 4 POLLEN out of the FLOWER
    static final double STEP_TIMEOUT_S = 5.0;  // safety: give up on any step after this
    static final double PARK_BY_SECONDS = 25.0; // skip to parking after this (AUTO is 30 s)
    // ==================================================

    /** true for the red OpMode, false for blue. */
    protected abstract boolean isRed();

    private Follower follower;
    private Pose start, shootNear, flower, shootFar, park;
    private Path toShootNear, toFlower, toShootFar, toPark, parkFromAnywhere;

    private enum State { TO_SHOOT_NEAR, SHOOT_NEAR, TO_FLOWER, INTAKE, TO_SHOOT_FAR, SHOOT_FAR, TO_PARK, DONE }
    private State state = State.TO_SHOOT_NEAR;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        start = pose(START);
        shootNear = pose(SHOOT_NEAR);
        flower = pose(FLOWER);
        shootFar = pose(SHOOT_FAR);
        park = pose(PARK);
        Pose control = pose(new double[]{FAR_CONTROL[0], FAR_CONTROL[1], 0});
        Pose secondShot = SECOND_BATCH_FAR_SIDE ? shootFar : shootNear;

        // Mirror of the visualizer project: constant / linear heading per path.
        toShootNear = line(start, shootNear).constant(shootNear.heading());
        toFlower = line(shootNear, flower).linear(shootNear.heading(), flower.heading());
        toShootFar = SECOND_BATCH_FAR_SIDE
                ? curve(flower, control, shootFar).linear(flower.heading(), shootFar.heading())
                : line(flower, shootNear).linear(flower.heading(), shootNear.heading());
        toPark = line(secondShot, park).linear(secondShot.heading(), park.heading());

        follower.setPose(start);
        telemetry.addData("Alliance", isRed() ? "RED" : "BLUE");
        telemetry.addData("Start", "x %.0f  y %.0f  heading %.0f", start.x(), start.y(), Math.toDegrees(start.heading()));
        telemetry.addData("Status", "Ready. Place robot at the start pose.");
    }

    @Override
    public void start() {
        follower.setPose(start);  // re-set in case the robot was nudged during init
        autoTimer.reset();
        follower.follow(toShootNear);
        // TODO: start spinning up the launcher here so it is ready on arrival
        stepTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();  // MUST run every loop

        boolean timedOut = stepTimer.seconds() > STEP_TIMEOUT_S;
        boolean driveDone = !follower.isBusy();

        // Parking is worth 5 pts: never let a slow step cost us the park.
        if (autoTimer.seconds() > PARK_BY_SECONDS && state != State.TO_PARK && state != State.DONE) {
            Pose here = follower.pose();
            parkFromAnywhere = line(here, park).linear(here.heading(), park.heading());
            follower.follow(parkFromAnywhere);
            next(State.TO_PARK);
        }

        switch (state) {
            case TO_SHOOT_NEAR:
                if (driveDone || timedOut) {
                    // TODO: feed the 4 preloads into the launcher
                    next(State.SHOOT_NEAR);
                }
                break;
            case SHOOT_NEAR:
                if (stepTimer.seconds() > SHOOT_SECONDS) {
                    // TODO: stop feeding; start the intake
                    follower.follow(toFlower);
                    next(State.TO_FLOWER);
                }
                break;
            case TO_FLOWER:
                if (driveDone || timedOut) next(State.INTAKE);
                break;
            case INTAKE:
                if (stepTimer.seconds() > INTAKE_SECONDS) {
                    // TODO: stop the intake; spin up the launcher
                    follower.follow(toShootFar);
                    next(State.TO_SHOOT_FAR);
                }
                break;
            case TO_SHOOT_FAR:
                if (driveDone || timedOut) {
                    // TODO: feed the 4 POLLEN into the launcher
                    next(State.SHOOT_FAR);
                }
                break;
            case SHOOT_FAR:
                if (stepTimer.seconds() > SHOOT_SECONDS) {
                    // TODO: stop the launcher
                    follower.follow(toPark);
                    next(State.TO_PARK);
                }
                break;
            case TO_PARK:
                if (driveDone || timedOut) {
                    follower.stop();
                    next(State.DONE);
                }
                break;
            case DONE:
                break;  // wait for the end of AUTO
        }

        Pose p = follower.pose();
        telemetry.addData("Alliance", isRed() ? "RED" : "BLUE");
        telemetry.addData("State", state);
        telemetry.addData("Auto time", "%.1f s", autoTimer.seconds());
        telemetry.addData("x / y / heading deg", "%.1f / %.1f / %.0f", p.x(), p.y(), Math.toDegrees(p.heading()));
    }

    /** Builds a Pose from a red {x, y, headingDeg}, rotated 180 degrees for blue. */
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
