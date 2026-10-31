package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Finds a HIVE CELL's AprilTags with the Limelight and reports how far off-center they are.
 *
 * BIOBUZZ tag IDs (Competition Manual 9.9, 36h11 family, 4 per CELL):
 *   30-33 red CELL, far side   |  34-37 red CELL, audience side
 *   38-41 blue CELL, audience  |  42-45 blue CELL, far side
 *
 * If no device named "limelight" is configured, available() is false and the auto skips aligning.
 */
public class TagAligner {

    // ================= TUNING SECTION =================
    public static int PIPELINE = 0;          // Limelight pipeline set up as AprilTag (36h11)
    public static long MAX_AGE_MS = 100;     // ignore results older than this
    // ==================================================

    private final Limelight3A limelight;

    public TagAligner(HardwareMap hw) {
        // tryGet returns null instead of crashing when the camera isn't configured yet.
        limelight = hw.tryGet(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(PIPELINE);
            limelight.start();
        }
    }

    public boolean available() { return limelight != null; }

    /**
     * Horizontal angle in degrees to the most centered tag with an ID in [minId, maxId].
     * Positive = the tag is to the RIGHT of the crosshair. Returns NaN if none is visible.
     */
    public double txTo(int minId, int maxId) {
        if (limelight == null) return Double.NaN;
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid() || r.getStaleness() > MAX_AGE_MS) return Double.NaN;

        double best = Double.NaN;
        for (LLResultTypes.FiducialResult tag : r.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (id < minId || id > maxId) continue;  // only our own CELL's tags
            double tx = tag.getTargetXDegrees();
            if (Double.isNaN(best) || Math.abs(tx) < Math.abs(best)) best = tx;
        }
        return best;
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
