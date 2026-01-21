/*
Second autonomous version (Backwards start position):

- Robot stays STILL at the start
- Reads AprilTag first and locks the plan
- Immediately shoots at 2400 RPM
- Then drives "forward" for 1 second at the end

IMPORTANT:
This version assumes the robot is oriented BACKWARDS on the field.
So to move FORWARD at the end, we drive in the OPPOSITE direction compared to the reference program.
*/

// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/autonomous/botAutonomous_BackStart2400.java
package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "botAutonomous BackStart 2400", group = "OB")
public class botAutonomousBack extends LinearOpMode {

    // =========================
    // Camera / AprilTag
    // =========================
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProc;

    private static final String WEBCAM_NAME = "Webcam 1";
    private static final String CAM_SERVO_NAME = "camServo";

    private static final double SERVO_FIRE_POS = 0.25;
    private static final double SERVO_REST_POS = 0.10;

    private static final double CAM_LEFT_POS = 0.0;
    private static final double CAM_CENTER_POS = 0.5;

    private static final boolean LOCK_CAMERA_EXPOSURE = false;
    private static final long EXPOSURE_MS = 12;
    private static final int GAIN_UNITS = 200;

    private static final Float DECIMATION = 2.0f;

    // =========================
    // Shooter (setVelocity)
    // =========================
    // NEW: 2400 RPM for this BackStart auto
    private static final double SHOOTER_TARGET_RPM = 2300.0;

    private static final double SHOOTER_TICKS_PER_REV = 28.0;

    private static final int LEFT_CMD_SIGN  = +1;
    private static final int RIGHT_CMD_SIGN = -1;

    private static final double LEFT_TRIM  = 1.00;
    private static final double RIGHT_TRIM = 1.00;

    private static final double AT_SPEED_TOL_RPM = 150.0;
    private static final double SPINUP_TIMEOUT_SEC = 2.0;

    // =========================
    // Scan behavior
    // =========================
    private static final double STATIONARY_SCAN_SECONDS = 0.9;

    // =========================
    // End drive behavior (ONLY wheel use after shooting)
    // =========================
    private static final double END_DRIVE_SECONDS = 1.0;
    private static final double END_DRIVE_POWER = 0.30;

    // =========================
    // Tag IDs you CARE about
    // =========================
    private static final int TAG1_ID = 21; // Plan 1
    private static final int TAG2_ID = 22; // Plan 2
    private static final int MIN_SEEN_FRAMES = 2;

    // =========================
    // Motors / Servos
    // =========================
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    private Servo camServo;
    private CRServo servoTrapLeft;
    private CRServo servoTrapRight;
    private Servo tipperServo;
    private Servo leftHolderServo;
    private Servo rightHolderServo;

    // Drivetrain
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;

    // Intake
    private DcMotorEx intakeMotor;

    // =========================
    // Pinpoint Odometry (kept)
    // =========================
    private GoBildaPinpointDriver pinpoint;
    private static final String PINPOINT_NAME = "pinpoint";

    private static final double PINPOINT_X_OFFSET_MM = -157.5;
    private static final double PINPOINT_Y_OFFSET_MM = 15.0;

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

        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        servoTrapLeft  = hardwareMap.get(CRServo.class, "servoTrapLeft");
        servoTrapRight = hardwareMap.get(CRServo.class, "servoTrapRight");
        tipperServo  = hardwareMap.get(Servo.class, "tipperServo");
        leftHolderServo  = hardwareMap.get(Servo.class, "leftHolderServo");
        rightHolderServo = hardwareMap.get(Servo.class, "rightHolderServo");

        // Drivetrain directions (kept EXACTLY like your reference auto)
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter directions for velocity logic
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setShooterRpm(0.0);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); // uncomment if needed

        tipperServo.setPosition(SERVO_REST_POS);
        leftHolderServo.setPosition(0.1);
        rightHolderServo.setPosition(0.40);

        // =========================
        // Pinpoint init (kept)
        // =========================
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        pinpoint.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(X_ENCODER_DIR, Y_ENCODER_DIR);
        pinpoint.resetPosAndIMU();
        sleep(350);
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

        FtcDashboard dash = FtcDashboard.getInstance();
        dash.startCameraStream(visionPortal, 30);
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        telemetry.addLine("Init complete. Press START.");
        telemetry.addData("Shooter Target RPM", "%.0f", SHOOTER_TARGET_RPM);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // =========================
        // AUTO START:
        // Scan first (robot stays still), decide plan, then SHOOT, then move forward 1 sec.
        // =========================
        stopDrive();
        camServo.setPosition(CAM_LEFT_POS);

        int seenTag1 = 0;
        int seenTag2 = 0;

        double bestZ1 = Double.POSITIVE_INFINITY;
        double bestZ2 = Double.POSITIVE_INFINITY;

        // --------
        // Phase A: STOPPED scan only
        // --------
        ElapsedTime scanTimer = new ElapsedTime();
        scanTimer.reset();

        while (opModeIsActive() && scanTimer.seconds() < STATIONARY_SCAN_SECONDS) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            List<AprilTagDetection> dets = tagProc.getDetections();
            int detCount = (dets == null) ? 0 : dets.size();

            if (dets != null) {
                for (AprilTagDetection d : dets) {
                    if (d == null) continue;

                    if (d.id == TAG1_ID) {
                        seenTag1++;
                        if (d.ftcPose != null) bestZ1 = Math.min(bestZ1, d.ftcPose.z);
                    } else if (d.id == TAG2_ID) {
                        seenTag2++;
                        if (d.ftcPose != null) bestZ2 = Math.min(bestZ2, d.ftcPose.z);
                    }
                }
            }

            telemetry.addData("Phase", "STOPPED Scan");
            telemetry.addData("Scan t", "%.2f / %.2f", scanTimer.seconds(), STATIONARY_SCAN_SECONDS);
            telemetry.addData("Detections", detCount);
            telemetry.addData("Seen Tag1 (21)", seenTag1);
            telemetry.addData("Seen Tag2 (22)", seenTag2);

            if (pose != null) {
                telemetry.addData("Odo X (mm)", "%.1f", pose.getX(DistanceUnit.MM));
                telemetry.addData("Odo Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
                telemetry.addData("Heading (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
            }

            telemetry.update();
            idle();
        }

        // Lock the decision NOW
        Integer chosenTagId = chooseBetweenTwoTags(seenTag1, seenTag2, bestZ1, bestZ2);

        telemetry.addData("Decision (locked)", chosenTagId == null ? "TAG3(default)" : (chosenTagId == TAG1_ID ? "TAG1" : "TAG2"));
        telemetry.update();

        // --------
        // Phase B: Immediately execute shooting plan at 2400 RPM
        // --------
        telemetry.addLine("Shooting NOW (2400 RPM)...");
        telemetry.update();

        if (chosenTagId == null) {
            doTag3Plan();
        } else if (chosenTagId == TAG1_ID) {
            doTag1Plan();
        } else if (chosenTagId == TAG2_ID) {
            doTag2Plan();
        } else {
            doTag3Plan();
        }

        // Stop shooter after plan
        setShooterRpm(0.0);

        // --------
        // Phase C: Move FORWARD for 1 second (robot is backwards, so opposite direction)
        // ONLY wheel movement in this entire auto.
        // --------
        telemetry.addLine("End move: forward 1 second (backwards-oriented robot)");
        telemetry.update();

        driveForward_BackStart(END_DRIVE_POWER);
        sleep((long) (END_DRIVE_SECONDS * 1000));
        stopDrive();

        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Decide plan:
     * - TAG1_ID if Tag1 seen confidently
     * - TAG2_ID if Tag2 seen confidently
     * - null if neither is confidently seen (fallback Tag3)
     */
    private Integer chooseBetweenTwoTags(int seen1, int seen2, double z1, double z2) {
        boolean ok1 = seen1 >= MIN_SEEN_FRAMES;
        boolean ok2 = seen2 >= MIN_SEEN_FRAMES;

        if (!ok1 && !ok2) return null;
        if (ok1 && !ok2) return TAG1_ID;
        if (!ok1 && ok2) return TAG2_ID;

        if (seen1 > seen2) return TAG1_ID;
        if (seen2 > seen1) return TAG2_ID;

        boolean z1ok = z1 < Double.POSITIVE_INFINITY;
        boolean z2ok = z2 < Double.POSITIVE_INFINITY;
        if (z1ok && z2ok) {
            return (z1 <= z2) ? TAG1_ID : TAG2_ID;
        }

        // Deterministic tie-break if no pose
        return TAG2_ID;
    }

    // =========================
    // Wheel helpers
    // =========================
    private void stopDrive() {
        motorFrontLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorBackRight.setPower(0.0);
    }

    /**
     * IMPORTANT:
     * Your reference auto uses NEGATIVE power in setAllDrivePower(pwr)
     * to move forward in that specific field orientation.
     *
     * This back-start auto wants "forward" to be the OPPOSITE direction,
     * since the robot is oriented backwards.
     *
     * So here we drive the opposite sign.
     */
    private void driveForward_BackStart(double pwr) {
        motorFrontLeft.setPower(+pwr);
        motorFrontRight.setPower(+pwr);
        motorBackLeft.setPower(+pwr);
        motorBackRight.setPower(+pwr);
    }

    // =========================
    // Trap / shooter routines
    // =========================
    private void runTrapServos() {
        servoTrapLeft.setPower(-0.6);
        servoTrapRight.setPower(0.6);
        sleep(1200);
        servoTrapLeft.setPower(0);
        servoTrapRight.setPower(0);
        sleep(1000);
        servoTrapLeft.setPower(-0.6);
        servoTrapRight.setPower(0.6);
        sleep(1200);
        servoTrapLeft.setPower(0);
        servoTrapRight.setPower(0);
    }

    /**
     * Spin shooter to target RPM (2400), wait until at speed (or timeout), then fire tipper.
     */
    private void shootBalls() {
        setShooterRpm(SHOOTER_TARGET_RPM);

        ElapsedTime spin = new ElapsedTime();
        spin.reset();
        while (opModeIsActive() && spin.seconds() < SPINUP_TIMEOUT_SEC) {
            double lRpm = getShooterRpmShootPositive(shooterLeft, LEFT_CMD_SIGN);
            double rRpm = getShooterRpmShootPositive(shooterRight, RIGHT_CMD_SIGN);
            boolean atSpeed = shooterAtSpeed(SHOOTER_TARGET_RPM, AT_SPEED_TOL_RPM);

            telemetry.addData("Shooter Target", "%.0f", SHOOTER_TARGET_RPM);
            telemetry.addData("Shooter L RPM", "%.0f", lRpm);
            telemetry.addData("Shooter R RPM", "%.0f", rRpm);
            telemetry.addData("At speed?", atSpeed);
            telemetry.update();

            if (atSpeed) break;
            idle();
        }

        tipperServo.setPosition(SERVO_FIRE_POS);
        sleep(350);
        tipperServo.setPosition(SERVO_REST_POS);

        setShooterRpm(0.0);
        sleep(150);
    }

    private void shootRight() {
        rightHolderServo.setPosition(0);
        sleep(600);
        runTrapServos();
        shootBalls();
    }

    private void shootLeft() {
        leftHolderServo.setPosition(.53);
        sleep(600);
        runTrapServos();
        shootBalls();
    }

    private void shootIntakeBall() {
        intakeMotor.setPower(-1.0);
        sleep(600);
        intakeMotor.setPower(0.0);
        runTrapServos();
        shootBalls();
    }

    private void shootingOrder1() {
        shootIntakeBall();
        shootLeft();
        shootRight();
        runTrapServos();
        shootBalls();
    }

    private void shootingOrder2() {
        shootRight();
        shootIntakeBall();
        shootLeft();
        runTrapServos();
        shootBalls();
    }

    private void shootingOrder3() {
        shootRight();
        shootLeft();
        shootIntakeBall();
        runTrapServos();
        shootBalls();
    }

    // =========================
    // Shooter velocity helpers
    // =========================
    private void setShooterRpm(double rpm) {
        double tps = rpmToTicksPerSec(rpm);

        double cmdLTps = LEFT_CMD_SIGN * tps * LEFT_TRIM;
        double cmdRTps = RIGHT_CMD_SIGN * tps * RIGHT_TRIM;

        shooterLeft.setVelocity(cmdLTps);
        shooterRight.setVelocity(cmdRTps);
    }

    private boolean shooterAtSpeed(double targetRpm, double tolRpm) {
        double tgt = Math.abs(targetRpm);

        double l = getShooterRpmShootPositive(shooterLeft, LEFT_CMD_SIGN);
        double r = getShooterRpmShootPositive(shooterRight, RIGHT_CMD_SIGN);

        return Math.abs(l - tgt) <= tolRpm && Math.abs(r - tgt) <= tolRpm;
    }

    private double getShooterRpmShootPositive(DcMotorEx m, int cmdSign) {
        double tpsRaw = m.getVelocity();
        double tpsShootPos = tpsRaw * cmdSign;
        return ticksPerSecToRpm(tpsShootPos);
    }

    private static double rpmToTicksPerSec(double rpm) {
        return (rpm * SHOOTER_TICKS_PER_REV) / 60.0;
    }

    private static double ticksPerSecToRpm(double tps) {
        return (tps * 60.0) / SHOOTER_TICKS_PER_REV;
    }

    // =========================
    // TAG PLANS
    // =========================
    private void doTag1Plan() {
        telemetry.addLine("Running TAG 1 plan.");
        telemetry.update();
        shootingOrder1();
    }

    private void doTag2Plan() {
        telemetry.addLine("Running TAG 2 plan.");
        telemetry.update();
        shootingOrder2();
    }

    private void doTag3Plan() {
        telemetry.addLine("Running TAG 3 DEFAULT plan.");
        telemetry.update();
        shootingOrder3();
    }
}
