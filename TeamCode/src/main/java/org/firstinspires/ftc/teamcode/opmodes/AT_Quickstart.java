package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "AT_Quickstart")
public class AT_Quickstart extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor tagProc;

    // === EDIT THESE ===
    private static final String WEBCAM_NAME = "Webcam 1"; // Match your RC config exactly

    // Optional: lock exposure/gain for stable detection in variable lighting
    private static final boolean LOCK_CAMERA_EXPOSURE = false;
    private static final long   EXPOSURE_MS           = 12;   // Tune for your lighting
    private static final int    GAIN_UNITS            = 200;  // Tune for your lighting

    // Optional: decimation (trade detail for speed). 1.0 = full detail, 2.0 ~ faster.
    private static final Float  DECIMATION            = 2.0f;

    @Override
    public void runOpMode() {
        // 1) Build the AprilTag processor using SDK defaults (no custom tag sizes here).
        //    This avoids the addTag(...) overload differences between SDKs.
        tagProc = AprilTagProcessor.easyCreateWithDefaults();

        if (DECIMATION != null) {
            tagProc.setDecimation(DECIMATION);
        }

        // 2) Build VisionPortal:
        //    C270 works best at 640x480 in the SDK because intrinsics are provided at that res.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(tagProc)
                .enableLiveView(true)
                .build();

        // Wait until video is streaming before touching camera controls.
        while (!isStopRequested()
                && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(10);
            idle();
        }

        // 3) (Optional) Lock exposure & gain via VisionPortal camera controls.
        if (LOCK_CAMERA_EXPOSURE && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exp = visionPortal.getCameraControl(ExposureControl.class);
            if (exp != null) {
                exp.setMode(ExposureControl.Mode.Manual);
                exp.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
            }
            GainControl gain = visionPortal.getCameraControl(GainControl.class);
            if (gain != null) {
                int clamped = Math.max(gain.getMinGain(), Math.min(GAIN_UNITS, gain.getMaxGain()));
                gain.setGain(clamped);
            }
        }

        // 4) Dashboard streaming + merged telemetry (RC + Dashboard)
        FtcDashboard dash = FtcDashboard.getInstance();
        dash.startCameraStream(visionPortal, 30);
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        telemetry.addLine("AprilTag init complete. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> dets = tagProc.getDetections();

            if (!dets.isEmpty()) {
                // Pick a "best" detection by smallest forward range (z) with a valid ftcPose.
                AprilTagDetection best = null;
                for (AprilTagDetection d : dets) {
                    if (d.ftcPose == null) continue;
                    if (best == null || d.ftcPose.z < best.ftcPose.z) best = d;
                }

                telemetry.addData("Detections", dets.size());
                StringBuilder ids = new StringBuilder();
                for (AprilTagDetection d : dets) {
                    ids.append(d.id).append(" ");
                }
                telemetry.addData("IDs", ids.toString().trim());

                if (best != null && best.ftcPose != null) {
                    telemetry.addLine("--- Best ---");
                    telemetry.addData("ID", best.id);
                    telemetry.addData("Z (forward, m)", "%.3f", best.ftcPose.z);
                    telemetry.addData("X (left+, m)", "%.3f", best.ftcPose.x);
                    telemetry.addData("Y (up+, m)", "%.3f", best.ftcPose.y);
                    telemetry.addData("Yaw (deg)",   "%.1f", best.ftcPose.yaw);
                    telemetry.addData("Pitch (deg)", "%.1f", best.ftcPose.pitch);
                    telemetry.addData("Roll (deg)",  "%.1f", best.ftcPose.roll);
                } else {
                    telemetry.addLine("Detections present, but no reliable ftcPose yet.");
                }
            } else {
                telemetry.addLine("No tags.");
            }

            telemetry.update();
            idle();
        }

        // 5) Clean up
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
