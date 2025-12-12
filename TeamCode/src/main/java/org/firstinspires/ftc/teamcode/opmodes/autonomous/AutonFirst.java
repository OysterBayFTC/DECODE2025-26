package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "AutonFirst", group = "OB")
public class AutonFirst extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor tagProc;

    // === EDIT THESE ===
    private static final String WEBCAM_NAME = "Webcam 1";     // Match RC config exactly
    private static final String CAM_SERVO_NAME = "camServo";  // Match your RC config exactly

    // Servo positions (placeholders, you will tune)
    private static final double CAM_LEFT_POS = 0.0;
    private static final double CAM_CENTER_POS = 0.5;

    // Optional: lock exposure/gain for stable detection in variable lighting
    private static final boolean LOCK_CAMERA_EXPOSURE = false;
    private static final long   EXPOSURE_MS           = 12;
    private static final int    GAIN_UNITS            = 200;

    // Optional: decimation (trade detail for speed). 1.0 = full detail, 2.0 ~ faster.
    private static final Float  DECIMATION            = 2.0f;

    // Scan/decision timing (placeholders, you will tune)
    private static final double DRIVE_AND_SCAN_SECONDS = 1.5; // drive forward while scanning
    private static final double POST_DECISION_PAUSE_SEC = 0.1;

    private Servo camServo;

    @Override
    public void runOpMode() {
        // === Hardware ===
        camServo = hardwareMap.get(Servo.class, CAM_SERVO_NAME);
        camServo.setPosition(CAM_CENTER_POS);

        // === AprilTag init ===
        tagProc = AprilTagProcessor.easyCreateWithDefaults();
        if (DECIMATION != null) tagProc.setDecimation(DECIMATION);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(tagProc)
                .enableLiveView(true)
                .build();

        while (!isStopRequested()
                && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(10);
            idle();
        }

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

        // Dashboard streaming + merged telemetry
        FtcDashboard dash = FtcDashboard.getInstance();
        dash.startCameraStream(visionPortal, 30);
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        telemetry.addLine("AprilTag init complete. Press START.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // =========================================================
        // START OF AUTO
        // 1) Immediately swing camera left
        // 2) Begin driving forward
        // 3) During a short window, try to lock in the best tag ID
        // =========================================================

        camServo.setPosition(CAM_LEFT_POS);

        // Start moving forward (you implement this)
        startDriveForward();

        Integer chosenTagId = null;
        AprilTagDetection chosenDet = null;

        resetRuntime();
        while (opModeIsActive() && getRuntime() < DRIVE_AND_SCAN_SECONDS) {
            AprilTagDetection best = getBestDetection(tagProc.getDetections());

            if (best != null) {
                // Keep the most recent "best" detection as our candidate
                chosenDet = best;
                chosenTagId = best.id;
            }

            // Telemetry while scanning
            telemetry.addData("Phase", "Drive+Scan");
            telemetry.addData("CandidateTag", chosenTagId == null ? "none" : chosenTagId);
            if (chosenDet != null && chosenDet.ftcPose != null) {
                telemetry.addData("Z (m)", "%.3f", chosenDet.ftcPose.z);
                telemetry.addData("X (m)", "%.3f", chosenDet.ftcPose.x);
                telemetry.addData("Yaw (deg)", "%.1f", chosenDet.ftcPose.yaw);
            }
            telemetry.update();
            idle();
        }

        // Stop driving once scan window ends
        stopDrive();

        // Optional small pause so actions feel deterministic
        sleep((long) (POST_DECISION_PAUSE_SEC * 1000));

        // =========================================================
        // BRANCH AUTONOMOUS BASED ON TAG
        // If no tag found, do default.
        // Replace IDs and actions with your actual strategy.
        // =========================================================

        telemetry.addLine("Decision complete.");
        telemetry.addData("ChosenTag", chosenTagId == null ? "none" : chosenTagId);
        telemetry.update();

        if (chosenTagId == null) {
            doNoTagPlan();
        } else {
            switch (chosenTagId) {
                case 1:
                    doTag1Plan();
                    break;
                case 2:
                    doTag2Plan();
                    break;
                case 3:
                    doTag3Plan();
                    break;
                default:
                    doUnknownTagPlan(chosenTagId);
                    break;
            }
        }

        // Cleanup
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Picks a "best" detection by smallest forward range (z) with a valid ftcPose.
     */
    private AprilTagDetection getBestDetection(List<AprilTagDetection> dets) {
        if (dets == null || dets.isEmpty()) return null;

        AprilTagDetection best = null;
        for (AprilTagDetection d : dets) {
            if (d == null || d.ftcPose == null) continue;
            if (best == null || d.ftcPose.z < best.ftcPose.z) best = d;
        }
        return best;
    }

    // =========================================================
    // DRIVETRAIN PLACEHOLDERS
    // Replace with your RoadRunner, mecanum drive class, or motor control.
    // =========================================================

    private void startDriveForward() {
        // TODO: start driving forward
        // Examples:
        // - set motor powers
        // - call your drive.followTrajectoryAsync(...)
        // - set a state in your own drive loop
    }

    private void stopDrive() {
        // TODO: stop drivetrain
    }

    // =========================================================
    // TAG PLANS (PLACEHOLDERS)
    // Put your real auto actions here.
    // =========================================================

    private void doNoTagPlan() {
        telemetry.addLine("Running NO TAG plan.");
        telemetry.update();
        // TODO: your default autonomous routine
        sleep(250);
    }

    private void doTag1Plan() {
        telemetry.addLine("Running TAG 1 plan.");
        telemetry.update();
        // TODO: actions for tag 1
        sleep(250);
    }

    private void doTag2Plan() {
        telemetry.addLine("Running TAG 2 plan.");
        telemetry.update();
        // TODO: actions for tag 2
        sleep(250);
    }

    private void doTag3Plan() {
        telemetry.addLine("Running TAG 3 plan.");
        telemetry.update();
        // TODO: actions for tag 3
        sleep(250);
    }

    private void doUnknownTagPlan(int tagId) {
        telemetry.addData("Running UNKNOWN TAG plan for", tagId);
        telemetry.update();
        // TODO: fallback for unexpected tag IDs
        sleep(250);
    }
}
