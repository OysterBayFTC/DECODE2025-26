package org.firstinspires.ftc.teamcode.opmodes;

    // File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/LimelightTag36Light.java


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

    /*
     * Limelight 3A + goBILDA RGB Indicator Light test.
     *
     * Follows ONLY AprilTag ID 36 (any other tags in view are ignored) and shows on the
     * Driver Station how far the camera is from the tag and at what angle. The indicator
     * light changes color with distance:
     *
     *   tag not seen, or farther than 2.5 m  ->  black (light off)
     *   2.5 m down to 0.3 m                  ->  red, orange, yellow, sage, green, azure, blue, indigo, violet
     *   0.3 m or closer                      ->  white
     *
     * Robot Configuration:
     *   - Limelight 3A (plugged into USB, listed with the webcams)   name: "limelight"
     *   - goBILDA RGB Indicator Light on a servo port, type Servo     name: "rgbLight"
     *
     * Limelight pipeline (set up in the Limelight web interface):
     *   - PIPELINE_INDEX must be an AprilTag pipeline (family 36h11)
     *   - Set the tag size to the printed tag's size, or the distance will be off
     *   - If the tag is detected but Distance says "unavailable", the pipeline is not sending
     *     3D pose data: check its tag size and 3D settings
     */
    @TeleOp(name = "Limelight Tag 36 Light", group = "Test")
    public class LimelightTag36Light extends LinearOpMode {

        // =========================
        // Hardware names (must match the Robot Configuration)
        // =========================
        private static final String LIMELIGHT_NAME = "limelight";
        private static final String LIGHT_NAME = "rgbLight";

        // =========================
        // AprilTag
        // =========================
        private static final int PIPELINE_INDEX = 0;   // Limelight pipeline set up for AprilTags
        private static final int TARGET_TAG_ID = 36;   // the only tag this program follows

        // A result older than this means the Limelight has stopped sending data
        private static final long MAX_RESULT_AGE_MS = 250;

        // =========================
        // Distance -> light color
        // =========================
        private static final double FAR_DISTANCE_M  = 2.5;  // farther than this: black (off)
        private static final double NEAR_DISTANCE_M = 0.3;  // this close or closer: white

        // goBILDA RGB Indicator Light servo positions (SDK default servo range is 600-2400 us).
        // Positions from red (~1100 us) to violet (~1900 us) blend smoothly through the spectrum.
        private static final double LIGHT_OFF    = 0.0;
        private static final double LIGHT_RED    = 0.277;
        private static final double LIGHT_VIOLET = 0.722;
        private static final double LIGHT_WHITE  = 1.0;

        // Color names from red to violet, evenly spaced, for the Driver Station readout
        private static final String[] SPECTRUM_NAMES =
                { "Red", "Orange", "Yellow", "Sage", "Green", "Azure", "Blue", "Indigo", "Violet" };

        private static final double INCHES_PER_METER = 39.3701;

        private Limelight3A limelight;
        private Servo light;

        public void runOpMode() {
            limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
            light = hardwareMap.get(Servo.class, LIGHT_NAME);
            light.setPosition(LIGHT_OFF);

            // Send telemetry often so the Driver Station keeps up with the camera
            telemetry.setMsTransmissionInterval(11);

            // The Limelight takes a while to boot, so keep asking until it switches pipelines
            while (opModeInInit() && !limelight.pipelineSwitch(PIPELINE_INDEX)) {
                telemetry.addData("Status", "Waiting for the Limelight to respond");
                telemetry.update();
                sleep(500);
            }

            // Without start(), getLatestResult() always returns null
            limelight.start();

            telemetry.addData("Status", "Ready. Press Play, then point the Limelight at AprilTag %d", TARGET_TAG_ID);
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                showTag(limelight.getLatestResult());
            }

            light.setPosition(LIGHT_OFF);
            limelight.stop();
        }

        /** Finds tag 36 in the latest result, sets the light, and reports on the Driver Station. */
        private void showTag(LLResult result) {
            if (result == null || result.getStaleness() > MAX_RESULT_AGE_MS) {
                light.setPosition(LIGHT_OFF);
                telemetry.addData("Status", "Waiting for Limelight data");
                telemetry.addData("Light", lightName(LIGHT_OFF));
                telemetry.update();
                return;
            }

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            LLResultTypes.FiducialResult tag = findTargetTag(tags);
            double distanceM = (tag == null) ? Double.NaN : distanceMeters(tag);

            double lightPosition = lightPositionFor(distanceM);
            light.setPosition(lightPosition);

            if (tag == null) {
                telemetry.addData("Status", "AprilTag %d not detected", TARGET_TAG_ID);
            } else {
                telemetry.addData("Status", "AprilTag %d detected", TARGET_TAG_ID);
                if (Double.isNaN(distanceM)) {
                    telemetry.addData("Distance", "unavailable (check the tag size and 3D settings in the Limelight pipeline)");
                } else {
                    telemetry.addData("Distance", "%.2f m (%.1f in)", distanceM, distanceM * INCHES_PER_METER);
                }
                telemetry.addData("Angle", angleText(tag.getTargetXDegrees()));
            }
            telemetry.addData("Light", lightName(lightPosition));
            telemetry.addData("Other tags in view", otherTagIds(tags));
            telemetry.update();
        }

        /** The tag 36 detection, or null. If more than one tag 36 is in view, the biggest (closest) one. */
        private static LLResultTypes.FiducialResult findTargetTag(List<LLResultTypes.FiducialResult> tags) {
            if (tags == null) return null;

            LLResultTypes.FiducialResult best = null;
            for (LLResultTypes.FiducialResult tag : tags) {
                if (tag.getFiducialId() != TARGET_TAG_ID) continue;
                if (best == null || tag.getTargetArea() > best.getTargetArea()) best = tag;
            }
            return best;
        }

        /**
         * Straight-line distance from the camera to the center of the tag, in meters.
         * Returns NaN if the Limelight did not send a 3D pose (the SDK reports a missing pose as all zeros).
         */
        private static double distanceMeters(LLResultTypes.FiducialResult tag) {
            double meters = length(tag.getCameraPoseTargetSpace());
            if (meters == 0) {
                // Same distance measured the other way around: the tag as seen from the camera
                meters = length(tag.getTargetPoseCameraSpace());
            }
            return meters > 0 ? meters : Double.NaN;
        }

        private static double length(Pose3D pose) {
            if (pose == null) return 0;
            Position p = pose.getPosition().toUnit(DistanceUnit.METER);
            return Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        }

        /**
         * Light position for a distance: black beyond FAR_DISTANCE_M, then red -> violet as the
         * camera gets closer, white at NEAR_DISTANCE_M or closer. NaN (no distance) is black.
         */
        private static double lightPositionFor(double distanceM) {
            if (Double.isNaN(distanceM) || distanceM > FAR_DISTANCE_M) return LIGHT_OFF;
            if (distanceM <= NEAR_DISTANCE_M) return LIGHT_WHITE;

            // 0.0 at the far end, 1.0 at the near end
            double closeness = (FAR_DISTANCE_M - distanceM) / (FAR_DISTANCE_M - NEAR_DISTANCE_M);
            return LIGHT_RED + closeness * (LIGHT_VIOLET - LIGHT_RED);
        }

        /** Name of the color the light shows at a position. */
        private static String lightName(double position) {
            if (position == LIGHT_OFF) return "Off (black)";
            if (position == LIGHT_WHITE) return "White";

            double step = (LIGHT_VIOLET - LIGHT_RED) / (SPECTRUM_NAMES.length - 1);
            int index = (int) Math.round((position - LIGHT_RED) / step);
            return SPECTRUM_NAMES[Math.max(0, Math.min(SPECTRUM_NAMES.length - 1, index))];
        }

        /**
         * Horizontal angle from the center of the camera's view (the Limelight crosshair) to the tag.
         * Limelight tx is positive when the tag is to the right.
         */
        private static String angleText(double txDegrees) {
            if (Math.abs(txDegrees) < 0.05) return "0.0 deg (centered)";
            return String.format("%.1f deg %s of center", Math.abs(txDegrees), txDegrees > 0 ? "right" : "left");
        }

        /** IDs of every tag in view except tag 36, or "none". */
        private static String otherTagIds(List<LLResultTypes.FiducialResult> tags) {
            StringBuilder ids = new StringBuilder();
            if (tags != null) {
                for (LLResultTypes.FiducialResult tag : tags) {
                    if (tag.getFiducialId() == TARGET_TAG_ID) continue;
                    if (ids.length() > 0) ids.append(", ");
                    ids.append(tag.getFiducialId());
                }
            }
            return ids.length() > 0 ? ids.toString() : "none";
        }
    }

