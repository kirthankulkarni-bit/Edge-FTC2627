package org.firstinspires.ftc.teamcode;

/**
 * Picks the flywheel speed from how far away the CELL is, so shots land from anywhere.
 *
 * Distance comes from ty: the Limelight's up/down angle to the CELL's AprilTags. With a fixed
 * camera, ty changes steadily as the robot moves closer or farther, so we don't need to
 * convert it to inches - we just record "at this ty, this RPM scores" and blend between rows.
 *
 * HOW TO FILL THE TABLE (robot + practice HIVE needed):
 *   1. Park at a spot, read ty on the Limelight Test OpMode.
 *   2. In Launcher Tuner, change Target RPM until shots land in the CELL every time.
 *   3. Write down {ty, rpm}. Repeat for 4-6 distances from close to far.
 * Rows must be sorted by ty, smallest first.
 */
public final class ShotTable {

    private ShotTable() {}

    // ================= TUNING SECTION =================
    // {ty degrees, flywheel RPM}. TODO: PLACEHOLDERS - replace with measured rows.
    // Typically the camera looks UP at the tags, so ty grows as the robot gets closer,
    // and closer shots need less speed.
    private static final double[][] ROWS = {
            {2, 4200},   // far
            {6, 3900},
            {10, 3600},
            {14, 3300},
            {18, 3000},  // close
    };
    // ==================================================

    /** RPM for a given ty. Outside the table it uses the nearest row (never guesses beyond it). */
    public static double rpmFor(double ty) {
        if (ty <= ROWS[0][0]) return ROWS[0][1];
        for (int i = 1; i < ROWS.length; i++) {
            if (ty <= ROWS[i][0]) {
                // Straight-line blend between the two rows around ty.
                double t = (ty - ROWS[i - 1][0]) / (ROWS[i][0] - ROWS[i - 1][0]);
                return ROWS[i - 1][1] + t * (ROWS[i][1] - ROWS[i - 1][1]);
            }
        }
        return ROWS[ROWS.length - 1][1];
    }
}
