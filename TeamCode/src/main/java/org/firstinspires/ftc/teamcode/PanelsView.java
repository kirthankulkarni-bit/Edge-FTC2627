package org.firstinspires.ftc.teamcode;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Panels dashboard helpers: live field drawing + telemetry in a browser.
 *
 * To open Panels: connect the laptop to the robot's Wi-Fi and go to http://192.168.43.1:8001
 *   - Field tab:      robot (blue), where it's been (green trail), planned path (orange)
 *   - Telemetry tab:  the same lines as the Driver Hub
 *   - Configurables:  live-edit the numbers in Launcher, Intake and TagAligner while the
 *                     robot runs (copy good values into the code afterwards - they reset
 *                     when the app restarts)
 * Panels uses Pedro's coordinates (inches, corner origin) because of setOffsets() below.
 */
public final class PanelsView {

    private PanelsView() {}

    public static final double ROBOT_RADIUS = 9;  // 18 in robot
    private static final int TRAIL_LENGTH = 150;  // ~3 s of history at 50 loops/s

    private static final FieldManager field = PanelsField.INSTANCE.getField();
    private static final Style ROBOT = new Style("", "#3F51B5", 0.75);
    private static final Style TRAIL = new Style("", "#4CAF50", 0.5);
    private static final Style PATH = new Style("", "#FF9800", 0.5);
    private static final Style TARGET = new Style("", "#E91E63", 0.75);

    private static final double[] trailX = new double[TRAIL_LENGTH];
    private static final double[] trailY = new double[TRAIL_LENGTH];
    private static int trailCount = 0, trailHead = 0;

    /** Call once in init(): Pedro coordinates, BIOBUZZ field picture, empty trail. */
    public static void init() {
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        field.setBackground(PanelsField.INSTANCE.getImages().getBIOBUZZ().getDARK());
        trailCount = 0;
        trailHead = 0;
    }

    /**
     * Sends every telemetry line to BOTH the Driver Hub and Panels.
     * Usage in an OpMode: telemetry = PanelsView.joined(telemetry);
     */
    public static Telemetry joined(Telemetry driverHub) {
        return new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), driverHub);
    }

    /** Draws the robot, its trail, the path it's following and (optionally) a target pose. Call every loop. */
    public static void draw(Pose robot, Path path, Pose target) {
        if (robot == null || Double.isNaN(robot.x()) || Double.isNaN(robot.y())) return;

        // Trail: a ring buffer of recent positions.
        trailX[trailHead] = robot.x();
        trailY[trailHead] = robot.y();
        trailHead = (trailHead + 1) % TRAIL_LENGTH;
        trailCount = Math.min(trailCount + 1, TRAIL_LENGTH);
        field.setStyle(TRAIL);
        for (int i = 1; i < trailCount; i++) {
            int a = (trailHead - trailCount + i - 1 + TRAIL_LENGTH) % TRAIL_LENGTH;
            int b = (a + 1) % TRAIL_LENGTH;
            field.moveCursor(trailX[a], trailY[a]);
            field.line(trailX[b], trailY[b]);
        }

        // Planned path: sample it at 20 points along its length (t from 0 to 1).
        if (path != null) {
            field.setStyle(PATH);
            Pose prev = path.get(0);
            for (int i = 1; i <= 20; i++) {
                Pose next = path.get(i / 20.0);
                field.moveCursor(prev.x(), prev.y());
                field.line(next.x(), next.y());
                prev = next;
            }
        }

        if (target != null) drawPose(target, TARGET);
        drawPose(robot, ROBOT);
        field.update();
    }

    /** A circle for the robot plus a line showing which way it faces. */
    private static void drawPose(Pose p, Style style) {
        field.setStyle(style);
        field.moveCursor(p.x(), p.y());
        field.circle(ROBOT_RADIUS);
        field.moveCursor(p.x(), p.y());
        field.line(p.x() + ROBOT_RADIUS * Math.cos(p.heading()), p.y() + ROBOT_RADIUS * Math.sin(p.heading()));
    }
}
