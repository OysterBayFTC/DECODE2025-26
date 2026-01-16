/*package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "AutonFirst", group = "OB")
public class AutonFirst extends LinearOpMode {

    // =========================
    // Camera / AprilTag
    // =========================
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProc;

    private static final String WEBCAM_NAME = "Webcam 1";
    private static final String CAM_SERVO_NAME = "camServo";

    private static final double CAM_LEFT_POS = 0.0;
    private static final double CAM_CENTER_POS = 0.5;

    private static final boolean LOCK_CAMERA_EXPOSURE = false;
    private static final long EXPOSURE_MS = 12;
    private static final int GAIN_UNITS = 200;

    private static final Float DECIMATION = 2.0f;

    // Drive+scan window
    private static final double DRIVE_AND_SCAN_SECONDS = 1.5;
    private static final double POST_DECISION_PAUSE_SEC = 0.1;

    // Simple forward power during scan
    private static final double SCAN_DRIVE_POWER = 0.30;

    private Servo camServo;

    // =========================
    // Drivetrain
    // =========================
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;
    private DcMotorEx motorPickUp;


    private CRServo servoTrapLeft;
    private CRServo servoTrapRight;

    // =========================
    // Pinpoint Odometry
    // =========================
    private GoBildaPinpointDriver pinpoint;

    // Match your RC config name for the Pinpoint I2C device
    private static final String PINPOINT_NAME = "pinpoint";

    // Your measured pod offsets (mm)
    // xOffset: sideways offset of the X (forward) pod, right is negative
    // yOffset: forward offset of the Y (strafe) pod, forward is positive :contentReference[oaicite:8]{index=8}
    private static final double PINPOINT_X_OFFSET_MM = -157.5;
    private static final double PINPOINT_Y_OFFSET_MM = 15.0;

    // Encoder direction defaults. You MUST verify on your bot:
    // X counts should increase when robot moves forward.
    // Y counts should increase when robot moves left. :contentReference[oaicite:9]{index=9}
    private static final GoBildaPinpointDriver.EncoderDirection X_ENCODER_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_ENCODER_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    @Override
    public void runOpMode() {
        // =========================
        // Hardware init
        // =========================
        camServo = hardwareMap.get(Servo.class, CAM_SERVO_NAME);
        camServo.setPosition(CAM_CENTER_POS);

        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorFrontLeft  = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackRight  = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        motorBackLeft   = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorPickUp = hardwareMap.get(DcMotorEx.class, "motorPickUp");


        servoTrapRight = hardwareMap.get(CRServo.class, "servoTrapRight");
        servoTrapLeft  = hardwareMap.get(CRServo.class, "servoTrapLeft");

        // Make +power = forward for all wheels (adjust if needed)
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorPickUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // =========================
        // Pinpoint init
        // =========================
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        // Required setup: offsets + pod resolution + encoder directions :contentReference[oaicite:10]{index=10}
        pinpoint.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(X_ENCODER_DIR, Y_ENCODER_DIR);

        // Recommended at the start of auto: reset position and recalibrate IMU (robot must be stationary) :contentReference[oaicite:11]{index=11}
        pinpoint.resetPosAndIMU();
        sleep(350);

        // Prime first read
        pinpoint.update();

        // =========================
        // AprilTag init
        // =========================
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

        telemetry.addLine("Init complete. Press START.");
        telemetry.addData("Pinpoint Status", String.valueOf(pinpoint.getDeviceStatus()));
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // =========================
        // START OF AUTO
        // =========================
        camServo.setPosition(CAM_LEFT_POS);

        Integer chosenTagId = null;
        AprilTagDetection chosenDet = null;

        // Begin moving forward while scanning
        setAllDrivePower(SCAN_DRIVE_POWER);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < DRIVE_AND_SCAN_SECONDS) {
            // You MUST call update() to get new data :contentReference[oaicite:12]{index=12}
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            AprilTagDetection best = getBestDetection(tagProc.getDetections());
            if (best != null) {
                chosenDet = best;
                chosenTagId = best.id;
            }

            telemetry.addData("Phase", "Drive+Scan");
            telemetry.addData("CandidateTag", chosenTagId == null ? "none" : chosenTagId);

            if (pose != null) {
                telemetry.addData("Odo X (mm)", "%.1f", pose.getX(DistanceUnit.MM));
                telemetry.addData("Odo Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
                telemetry.addData("Heading (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
            }

            if (chosenDet != null && chosenDet.ftcPose != null) {
                telemetry.addData("Tag Z (m)", "%.3f", chosenDet.ftcPose.z);
                telemetry.addData("Tag X (m)", "%.3f", chosenDet.ftcPose.x);
                telemetry.addData("Tag Yaw (deg)", "%.1f", chosenDet.ftcPose.yaw);
            }

            telemetry.addData("Pinpoint Status", String.valueOf(pinpoint.getDeviceStatus()));
            telemetry.update();
            idle();
        }

        stopDrive();
        sleep((long) (POST_DECISION_PAUSE_SEC * 1000));

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

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    /*
     * Picks a "best" detection by smallest forward range (z) with a valid ftcPose.
     */
/*
    private AprilTagDetection getBestDetection(List<AprilTagDetection> dets) {
        if (dets == null || dets.isEmpty()) return null;

        AprilTagDetection best = null;
        for (AprilTagDetection d : dets) {
            if (d == null || d.ftcPose == null) continue;
            if (best == null || d.ftcPose.z < best.ftcPose.z) best = d;
        }
        return best;
    }

    // =========================
    // DRIVETRAIN HELPERS
    // =========================
    private void setAllDrivePower(double pwr) {
        motorFrontLeft.setPower(pwr);
        motorFrontRight.setPower(pwr);
        motorBackLeft.setPower(pwr);
        motorBackRight.setPower(pwr);
    }

    private void stopDrive() {
        setAllDrivePower(0.0);
    }

    // =========================
    // TAG PLANS (PLACEHOLDERS)
    // =========================
    private void doNoTagPlan() {
        telemetry.addLine("Running NO TAG plan.");
        telemetry.update();
        sleep(250);
    }

    private void doTag1Plan() {
        telemetry.addLine("Running TAG 1 plan.");
        telemetry.update();
        sleep(250);

    }

    private void doTag2Plan() {
        telemetry.addLine("Running TAG 2 plan.");
        telemetry.update();
        sleep(250);
    }

    private void doTag3Plan() {
        telemetry.addLine("Running TAG 3 plan.");
        telemetry.update();
        sleep(250);
    }

    private void doUnknownTagPlan(int tagId) {
        telemetry.addData("Running UNKNOWN TAG plan for", tagId);
        telemetry.update();
        sleep(250);
    }
}
*/