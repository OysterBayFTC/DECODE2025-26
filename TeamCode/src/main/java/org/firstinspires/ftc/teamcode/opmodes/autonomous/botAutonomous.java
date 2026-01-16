/*
Two things you may need to flip after the first test:

1. If the robot strafes away from the Y target, change
PINPOINT_Y_INCREASES_WHEN_LEFT to the opposite value.

2. If the pickup motor spits out instead of intaking, reverse
intakeMotor direction (the commented line in init).
 */
// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/autonomous/botAutonomous.java
// 1820 Auton Regular
// 2400 Back corner
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

@Autonomous(name = "botAutonomous", group = "OB")
public class botAutonomous extends LinearOpMode {

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
    // Shooter (setVelocity) NEW
    // =========================
    // Your tuned target RPM for autonomous
    private static final double SHOOTER_TARGET_RPM = 1800.0;

    // goBILDA Yellow Jacket motor encoder is commonly 28 ticks/rev at the MOTOR shaft.
    // If RPM telemetry looks wrong, verify this for your exact motor/encoder.
    private static final double SHOOTER_TICKS_PER_REV = 28.0;

    // Sign convention (same as your working TeleOp/Velocity logic):
    // Both motors set Direction.FORWARD, then command RIGHT motor negative velocity so wheels spin inward.
    private static final int LEFT_CMD_SIGN  = +1;
    private static final int RIGHT_CMD_SIGN = -1;

    // Trim multipliers (start at 1.00 / 1.00, tune if one side consistently runs high/low)
    // Example: if LEFT reads ~+200 RPM high at a 2400 target, try LEFT_TRIM = 0.92–0.96.
    private static final double LEFT_TRIM  = 1.00;
    private static final double RIGHT_TRIM = 1.00;

    // Spin-up gating before firing (prevents shooting before wheels stabilize)
    private static final double AT_SPEED_TOL_RPM = 150.0;
    private static final double SPINUP_TIMEOUT_SEC = 2.0; // fail-safe so you still fire if sensor noise

    // =========================
    // Drive + scan behavior
    // =========================
    private static final double DRIVE_TOTAL_SECONDS = 2.5;
    private static final double PREMOVE_SECONDS = 1.0;
    private static final double STATIONARY_SCAN_SECONDS = 0.9;
    private static final double SCAN_DRIVE_POWER = 0.30;
    private static final double POST_DECISION_PAUSE_SEC = 0.1;

    // =========================
    // Tag IDs you CARE about
    // =========================
    private static final int TAG1_ID = 21; // Plan 1
    private static final int TAG2_ID = 22; // Plan 2
    private static final int MIN_SEEN_FRAMES = 2;

    // =========================
    // Positions (mm) (kept)
    // =========================
    private static final double BALL_PICK_UP_X1 = 125.0;
    private static final double BALL_PICK_UP_Y1 = 125.0;

    private static final double SHOOTING_X = 125.0;
    private static final double SHOOTING_Y = 125.0;

    // =========================
    // Pickup constants (kept)
    // =========================
    private static final double PICKUP_DRIVE_POWER = 0.30;
    private static final double PICKUP_MOTOR_POWER = 1.00;
    private static final double PICKUP_DURATION_SEC = 3.00;

    // =========================
    // Move-to constants (kept)
    // =========================
    private static final double POS_TOL_MM = 15.0;
    private static final double MOVE_TIMEOUT_SEC = 4.0;

    private static final double MOVE_FWD_POWER = 0.35;
    private static final double MOVE_STRAFE_POWER = 0.35;

    private static final boolean PINPOINT_Y_INCREASES_WHEN_LEFT = true;

    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    private Servo camServo;
    private CRServo servoTrapLeft;
    private CRServo servoTrapRight;
    private Servo tipperServo;
    private Servo leftHolderServo;
    private Servo rightHolderServo;

    // =========================
    // Drivetrain
    // =========================
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;

    // Intake
    private DcMotorEx intakeMotor;

    // =========================
    // Pinpoint Odometry
    // =========================
    private GoBildaPinpointDriver pinpoint;
    private static final String PINPOINT_NAME = "pinpoint";

    private static final double PINPOINT_X_OFFSET_MM = -157.5;
    private static final double PINPOINT_Y_OFFSET_MM = 15.0;

    private static final GoBildaPinpointDriver.EncoderDirection X_ENCODER_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_ENCODER_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // Optional TeleOp speed cap (off by default)
    private static final boolean USE_TELEOP_SPEED_MULT = false;
    private static final double TELEOP_SPEED_MULT = 0.55;

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

        // Directions (drivetrain kept exactly as your working version)
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Shooter directions (UPDATED for velocity logic)
        // Both FORWARD; we command RIGHT negative via RIGHT_CMD_SIGN.
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter encoder + velocity mode (NEW)
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setShooterRpm(0.0);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // If intake runs backwards, uncomment:
        // intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        tipperServo.setPosition(SERVO_REST_POS);
        leftHolderServo.setPosition(0.1);
        rightHolderServo.setPosition(0.40);

        // =========================
        // Pinpoint init
        // =========================
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        pinpoint.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(X_ENCODER_DIR, Y_ENCODER_DIR);

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
        telemetry.addData("Shooter Target RPM", "%.0f", SHOOTER_TARGET_RPM);
        telemetry.addData("Shooter Trim L/R", "%.3f / %.3f", LEFT_TRIM, RIGHT_TRIM);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // =========================
        // START OF AUTO
        // =========================
        camServo.setPosition(CAM_LEFT_POS);

        // We will decide tag while STOPPED (after moving a little), then finish the move, then shoot.
        int seenTag1 = 0;
        int seenTag2 = 0;

        // These Z values are OPTIONAL (only if ftcPose exists). We do NOT require ftcPose.
        double bestZ1 = Double.POSITIVE_INFINITY;
        double bestZ2 = Double.POSITIVE_INFINITY;

        // --------
        // Phase A: drive forward for PREMOVE_SECONDS
        // --------
        setAllDrivePower(SCAN_DRIVE_POWER);
        ElapsedTime tA = new ElapsedTime();
        tA.reset();
        while (opModeIsActive() && tA.seconds() < PREMOVE_SECONDS) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            telemetry.addData("Phase", "Drive (pre-scan)");
            telemetry.addData("t", "%.2f / %.2f", tA.seconds(), PREMOVE_SECONDS);
            if (pose != null) {
                telemetry.addData("Odo X (mm)", "%.1f", pose.getX(DistanceUnit.MM));
                telemetry.addData("Odo Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
                telemetry.addData("Heading (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
            }
            telemetry.update();
            idle();
        }
        stopDrive();

        // --------
        // Phase B: STOPPED scan for STATIONARY_SCAN_SECONDS
        // FIX: Count detections by ID EVEN if ftcPose is null.
        // --------
        ElapsedTime scanTimer = new ElapsedTime();
        scanTimer.reset();
        while (opModeIsActive() && scanTimer.seconds() < STATIONARY_SCAN_SECONDS) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            List<AprilTagDetection> dets = tagProc.getDetections();
            int detCount = (dets == null) ? 0 : dets.size();

            boolean saw22ThisLoop = false;

            if (dets != null) {
                for (AprilTagDetection d : dets) {
                    if (d == null) continue;

                    if (d.id == TAG2_ID) saw22ThisLoop = true;

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
            telemetry.addData("Detections this loop", detCount);
            telemetry.addData("Saw 22 this loop?", saw22ThisLoop);

            telemetry.addData("Seen Tag1 (21)", seenTag1);
            telemetry.addData("Seen Tag2 (22)", seenTag2);

            telemetry.addData("BestZ1 (m)", bestZ1 == Double.POSITIVE_INFINITY ? "n/a" : String.format("%.3f", bestZ1));
            telemetry.addData("BestZ2 (m)", bestZ2 == Double.POSITIVE_INFINITY ? "n/a" : String.format("%.3f", bestZ2));

            if (pose != null) {
                telemetry.addData("Odo X (mm)", "%.1f", pose.getX(DistanceUnit.MM));
                telemetry.addData("Odo Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
            }

            telemetry.update();
            idle();
        }

        // Lock the decision NOW (before moving again)
        Integer chosenTagId = chooseBetweenTwoTags(seenTag1, seenTag2, bestZ1, bestZ2);

        telemetry.addData("Decision (locked)", chosenTagId == null ? "TAG3(default)" : (chosenTagId == TAG1_ID ? "TAG1" : "TAG2"));
        telemetry.update();

        // --------
        // Phase C: finish remaining forward drive time so distance matches old behavior (2.5s total)
        // --------
        double remainingDrive = Math.max(0.0, DRIVE_TOTAL_SECONDS - PREMOVE_SECONDS);
        setAllDrivePower(SCAN_DRIVE_POWER);

        ElapsedTime tC = new ElapsedTime();
        tC.reset();
        while (opModeIsActive() && tC.seconds() < remainingDrive) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            telemetry.addData("Phase", "Drive (post-scan)");
            telemetry.addData("t", "%.2f / %.2f", tC.seconds(), remainingDrive);
            telemetry.addData("Decision (locked)", chosenTagId == null ? "TAG3(default)" : (chosenTagId == TAG1_ID ? "TAG1" : "TAG2"));
            if (pose != null) {
                telemetry.addData("Odo X (mm)", "%.1f", pose.getX(DistanceUnit.MM));
                telemetry.addData("Odo Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
            }
            telemetry.update();
            idle();
        }
        stopDrive();

        sleep((long) (POST_DECISION_PAUSE_SEC * 1000));

        // --------
        // Phase D: execute plan AFTER the movement
        // --------
        telemetry.addLine("Executing plan...");
        telemetry.addData("ChosenPlan", chosenTagId == null ? "TAG3 (default)" : (chosenTagId == TAG1_ID ? "TAG1" : "TAG2"));
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

        setShooterRpm(0.0);

        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Returns:
     *  - TAG1_ID if Tag1 is confidently seen
     *  - TAG2_ID if Tag2 is confidently seen
     *  - null if neither is confidently seen (fallback to Tag3 default)
     *
     * Decision rule:
     *  1) must be seen >= MIN_SEEN_FRAMES
     *  2) if both qualify: pick higher "seen" count; tie-break by closer Z IF available
     *  3) if tie and no pose: choose TAG2 (so it doesn't "fall into default")
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

        // Deterministic tie-break when pose isn't available:
        return TAG2_ID;
    }

    // =========================
    // PICKUP: drive + intake at same time (time-based) (kept)
    // =========================
    private void pickUp(double drivePower, double pickupPower, double durationSec) {
        setAllDrivePower(drivePower);
        intakeMotor.setPower(pickupPower);

        ElapsedTime t = new ElapsedTime();
        t.reset();

        while (opModeIsActive() && t.seconds() < durationSec) {
            idle();
        }

        stopDrive();
        intakeMotor.setPower(0);
    }

    private void pickUp() {
        pickUp(PICKUP_DRIVE_POWER, PICKUP_MOTOR_POWER, PICKUP_DURATION_SEC);
    }

    // =========================
    // MOVE TO POSITION: simple X then Y using Pinpoint (kept)
    // =========================
    private void moveToPosition(double targetXmm, double targetYmm) {
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();

        while (opModeIsActive() && timeout.seconds() < MOVE_TIMEOUT_SEC) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            if (pose == null) {
                telemetry.addLine("Pose is null");
                telemetry.update();
                idle();
                continue;
            }

            double x = pose.getX(DistanceUnit.MM);
            double y = pose.getY(DistanceUnit.MM);

            double xErr = targetXmm - x;
            double yErr = targetYmm - y;

            boolean atX = Math.abs(xErr) <= POS_TOL_MM;
            boolean atY = Math.abs(yErr) <= POS_TOL_MM;

            if (atX && atY) break;

            double forward = 0.0;
            double strafe = 0.0;

            if (!atX) {
                forward = (xErr > 0) ? MOVE_FWD_POWER : -MOVE_FWD_POWER;
            } else if (!atY) {
                double desired = (yErr > 0) ? MOVE_STRAFE_POWER : -MOVE_STRAFE_POWER;
                strafe = PINPOINT_Y_INCREASES_WHEN_LEFT ? desired : -desired;
            }

            driveMecanum(forward, strafe, 0.0);

            telemetry.addData("TargetX", "%.1f", targetXmm);
            telemetry.addData("TargetY", "%.1f", targetYmm);
            telemetry.addData("X", "%.1f", x);
            telemetry.addData("Y", "%.1f", y);
            telemetry.addData("xErr", "%.1f", xErr);
            telemetry.addData("yErr", "%.1f", yErr);
            telemetry.update();
            idle();
        }

        driveMecanum(0, 0, 0);
    }

    // =========================
    // DRIVETRAIN HELPERS (kept exactly)
    // =========================
    private void setAllDrivePower(double pwr) {
        motorFrontLeft.setPower(-pwr);
        motorFrontRight.setPower(-pwr);
        motorBackLeft.setPower(-pwr);
        motorBackRight.setPower(-pwr);
    }

    private void stopDrive() {
        setAllDrivePower(0.0);
    }

    // forward, strafe, turn in [-1..1]
    private void driveMecanum(double forward, double strafe, double turn) {
        if (USE_TELEOP_SPEED_MULT) {
            forward *= TELEOP_SPEED_MULT;
            strafe  *= TELEOP_SPEED_MULT;
            turn    *= TELEOP_SPEED_MULT;
        }

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        motorFrontLeft.setPower(fl / max);
        motorFrontRight.setPower(fr / max);
        motorBackLeft.setPower(bl / max);
        motorBackRight.setPower(br / max);
    }

    // =========================
    // Trap / shooter routines (UPDATED shooter to setVelocity)
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
     * Spin shooter to target RPM (with trim), wait until at speed (or timeout), then fire tipper.
     * This replaces your old setPower-based shootBalls().
     */
    private void shootBalls() {
        // 1) Spin up
        setShooterRpm(SHOOTER_TARGET_RPM);

        // 2) Wait until at speed or timeout
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

        // 3) Fire
        tipperServo.setPosition(SERVO_FIRE_POS);
        sleep(350); // keep similar to your old firing window (adjust if needed)
        tipperServo.setPosition(SERVO_REST_POS);

        // 4) Spin down (or keep spinning if you want faster multi-shot sequences)
        setShooterRpm(0.0);
        sleep(150); // small settle
    }

    private void shootRight() {
        rightHolderServo.setPosition(0);
        // .45
        sleep(600);
        runTrapServos();
        shootBalls();
    }

    private void shootLeft() {
        leftHolderServo.setPosition(.53);
        // .22
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
    // Shooter velocity helpers (NEW)
    // =========================
    private void setShooterRpm(double rpm) {
        // Convert RPM -> ticks/sec
        double tps = rpmToTicksPerSec(rpm);

        // Apply sign + trim
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
        double tpsRaw = m.getVelocity();       // ticks/sec raw
        double tpsShootPos = tpsRaw * cmdSign; // flip so "shooting direction" is positive
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
        driveMecanum(0.0, 0.35, 0.0); sleep(350); driveMecanum(0.0, 0.0, 0.0);
    }

    private void doTag2Plan() {
        telemetry.addLine("Running TAG 2 plan.");
        telemetry.update();
        shootingOrder2();
        driveMecanum(0.0, 0.35, 0.0); sleep(350); driveMecanum(0.0, 0.0, 0.0);
    }

    private void doTag3Plan() {
        telemetry.addLine("Running TAG 3 DEFAULT plan.");
        telemetry.update();
        shootingOrder3();
        driveMecanum(0.0, 0.35, 0.0); sleep(350); driveMecanum(0.0, 0.0, 0.0);
    }
}
