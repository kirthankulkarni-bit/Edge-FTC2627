package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/** Shows what the Limelight sees on the Driver Hub. Does not move the robot. */
@TeleOp(name = "Limelight Test", group = "Test")
public class LimelightTest extends LinearOpMode {

    // ================= TUNING SECTION =================
    static final int PIPELINE = 0;        // slot set up as AprilTag in the Limelight web page
    static final int POLL_HZ = 100;       // how often to ask the camera for new data
    static final long MAX_AGE_MS = 100;   // ignore results older than this
    // ==================================================

    @Override
    public void runOpMode() {
        // Name must match the Driver Hub robot configuration exactly.
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(POLL_HZ);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();  // without this, getLatestResult() never updates

        telemetry.addData("Status", "Ready. Point the camera at an AprilTag.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            // A null or invalid result just means "no target right now", not an error.
            if (result != null && result.isValid() && result.getStaleness() < MAX_AGE_MS) {
                telemetry.addData("tx / ty / ta", "%.1f / %.1f / %.2f",
                        result.getTx(), result.getTy(), result.getTa());

                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    telemetry.addData("Tag", tag.getFiducialId());
                }

                Pose3D bot = result.getBotpose();  // meters, FTC field coordinates
                if (bot != null) {
                    telemetry.addData("Botpose (m)", "x %.2f  y %.2f",
                            bot.getPosition().x, bot.getPosition().y);
                }
            } else {
                telemetry.addData("Target", "none");
            }
            telemetry.update();
        }
        limelight.stop();  // stop polling when the OpMode ends
    }
}
