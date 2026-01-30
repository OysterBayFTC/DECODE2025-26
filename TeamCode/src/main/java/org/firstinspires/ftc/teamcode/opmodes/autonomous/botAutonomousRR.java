/*
Road Runner + Pinpoint version of botAutonomous.

What changed vs your old botAutonomous:
- Drivetrain motor mapping + driveMecanum + setAllDrivePower + moveToPosition were removed.
- Uses PinpointDrive (your RoadRunner drive) + Actions to move.
- Keeps AprilTag scan logic and shooter/mechanism logic.
- Tag plans now run RoadRunner movement actions.

What you must tune:
- INITIAL_POSE (where your robot starts in RR coordinates)
- PREMOVE_FORWARD_IN / POSTMOVE_FORWARD_IN
- TAG1 / TAG2 / TAG3 target points and turns (converted from your mm targets)
*/

package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import java.util.concurrent.Callable;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "botAutonomousRR", group = "OB")
public class botAutonomousRR extends LinearOpMode {

    // =========================
    // Road Runner start + distances YOU WILL TUNE
    // =========================
    // Start pose in INCHES and RADIANS (Road Runner units).
    // If you have no idea yet, start with (0,0,0) and physically place robot consistently.
    private static final Pose2d INITIAL_POSE = new Pose2d(0.0, 0.0, 0.0);


    // These replace your old "drive forward for N seconds at power"
    // Start conservative (like 6-12 inches), test, then adjust.
    private static final double PREMOVE_FORWARD_IN = -10.0;
    private static final double pickUpPosition10 = 11.0;
    // take this out after you run the rotate test
    private static final boolean TURN_TEST_ONLY = true;


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
    private static final Pose2d shootingPosition = new Pose2d(-42, 0, Math.toRadians(0));
    Pose2d target = new Pose2d(-45.7, 21.2, Math.toRadians(-141));

    Pose2d pickUpPositionEnd1 = new Pose2d(-26.4, 36.6, Math.toRadians(-141));

    private static final Pose2d pickUpPosition2 = new Pose2d(-56, 42.6, Math.toRadians(-144));
    private static final Pose2d pickUpPosition3 = new Pose2d(-70, 61.6, Math.toRadians(-144));


    // =========================
    // Shooter (setVelocity)
    // =========================
    private static final double SHOOTER_TARGET_RPM = 1800.0;
    private static final double SHOOTER_TICKS_PER_REV = 28.0;

    private static final int LEFT_CMD_SIGN = +1;
    private static final int RIGHT_CMD_SIGN = -1;

    private static final double LEFT_TRIM = 1.00;
    private static final double RIGHT_TRIM = 1.00;

    private static final double AT_SPEED_TOL_RPM = 150.0;
    private static final double SPINUP_TIMEOUT_SEC = 2.0;
    private DcMotorEx shooterMotor;

    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    // Intake
    private DcMotorEx intakeMotor;

    // Servos
    private Servo camServo;
    private CRServo servoTrapLeft;
    private CRServo servoTrapRight;
    private Servo tipperServo;
    private Servo leftHolderServo;
    private Servo rightHolderServo;

    // =========================
    // Drive + scan behavior
    // =========================
    private static final double STATIONARY_SCAN_SECONDS = 0.9;

    // =========================
    // Tag IDs you CARE about
    // =========================
    private static final int TAG1_ID = 21; // Plan 1
    private static final int TAG2_ID = 22; // Plan 2
    private static final int MIN_SEEN_FRAMES = 2;

    // =========================
    // Your Tag1 plan points from old code were in mm:
    // moveToPosition(-620,-400); moveToPosition(-795,59);
    // Convert mm -> inches: in = mm / 25.4
    // -620mm  = -24.409in
    // -400mm  = -15.748in
    // -795mm  = -31.299in
    //  59mm   =  2.323in
    //
    // IMPORTANT:
    // These coordinates only make sense if your RR pose origin matches your old
    // pinpoint reset origin and axes.
    // =========================
    private static final Vector2d TAG1_POINT_A = new Vector2d(-24.409, -15.748);
    private static final Vector2d TAG1_POINT_B = new Vector2d(-31.299, 2.323);

    // This replaces your old timed turn (driveMecanum turn 0.6 for 1000ms).
    // Start with 90deg and adjust.
    // Tag2/3 had a timed strafe for 700ms at 0.35.
    // Replace with a distance and tune.
    private static final double TAG23_STRAFE_LEFT_IN = 10.0;

    @Override
    public void runOpMode() {

        // =========================
        // Hardware init (non-drive)
        // =========================
        camServo = hardwareMap.get(Servo.class, CAM_SERVO_NAME);
        camServo.setPosition(CAM_CENTER_POS);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        servoTrapLeft = hardwareMap.get(CRServo.class, "servoTrapLeft");
        servoTrapRight = hardwareMap.get(CRServo.class, "servoTrapRight");
        tipperServo = hardwareMap.get(Servo.class, "tipperServo");
        leftHolderServo = hardwareMap.get(Servo.class, "leftHolderServo");
        rightHolderServo = hardwareMap.get(Servo.class, "rightHolderServo");

        // Shooter directions
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Shooter encoder + velocity mode
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // setShooterRpm(0.0);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tipperServo.setPosition(SERVO_REST_POS);
        leftHolderServo.setPosition(0.1);
        rightHolderServo.setPosition(0.40);

        // =========================
        // Road Runner drive init (this handles drivetrain + pinpoint)
        // =========================
        PinpointDrive drive = new PinpointDrive(hardwareMap, INITIAL_POSE);

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
        telemetry.addData("Shooter Target RPM", "%.0f", SHOOTER_TARGET_RPM);
        telemetry.update();
        while (!isStarted() && !isStopRequested()) {
            drive.updatePoseEstimate();
            telemetry.addData("pose", "(%.2f, %.2f, %.1fdeg)",
                    drive.pose.position.x,
                    drive.pose.position.y,
                    Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }


        waitForStart();
        if (isStopRequested()) return;

        // =========================
        // START OF AUTO
        // =========================
        camServo.setPosition(CAM_LEFT_POS);


// After waitForStart():
        drive.pose = INITIAL_POSE;          // or drive.setPoseEstimate(INITIAL_POSE) if your class has that
        drive.updatePoseEstimate();         // force one update so you see any heading jump immediately

        Action preMove = drive.actionBuilder(drive.pose)
                .lineToX(-35)
                .build();

        Actions.runBlocking(preMove);


        sleep(100);
        Action pickUpMove = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(target, Math.toRadians(0))
                .build();

        Actions.runBlocking(pickUpMove);
        sleep(100);
        Action pickUpMoveEnd = drive.actionBuilder(drive.pose)
                .lineToX(-26.5)
                .lineToY(36.6)


                .build();

        Actions.runBlocking(pickUpMoveEnd);
        sleep(100);

        Action postMove = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(shootingPosition, Math.toRadians(0))
                .build();

        Actions.runBlocking(postMove);

        Action pickUp2 = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(pickUpPosition2, Math.toRadians(-144))
                .build();

        Actions.runBlocking(pickUp2);
        Action pickUpMoveEnd2 = drive.actionBuilder(drive.pose)
                .lineToX(-37.5)
                .lineToY(56.3)

                .build();
        Action postMove2 = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(shootingPosition, Math.toRadians(0))
                .build();

        Actions.runBlocking(postMove2);

        Actions.runBlocking(pickUpMoveEnd2);
        Action pickUp3 = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(pickUpPosition3, Math.toRadians(-144))
                .build();

        Actions.runBlocking(pickUp3);


        Action pickUpMoveEnd3 = drive.actionBuilder(drive.pose)
                .lineToX(-53.5)
                .lineToY(73.8)

                .build();

        Actions.runBlocking(pickUpMoveEnd3);

        Action postMove3 = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(shootingPosition, Math.toRadians(0))
                .build();

        Actions.runBlocking(postMove3);




/*
        // =========================
        // STOPPED scan phase
        // =========================
        int seenTag1 = 0;
        int seenTag2 = 0;

        double bestZ1 = Double.POSITIVE_INFINITY;
        double bestZ2 = Double.POSITIVE_INFINITY;

        ElapsedTime scanTimer = new ElapsedTime();
        scanTimer.reset();

        while (opModeIsActive() && scanTimer.seconds() < STATIONARY_SCAN_SECONDS) {
            // keep pose estimate fresh while stopped
            drive.updatePoseEstimate();

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
            telemetry.addData("RR Pose", "(%.2f, %.2f, %.1fdeg)",
                    drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();

            idle();
        }

        Integer chosenTagId = chooseBetweenTwoTags(seenTag1, seenTag2, bestZ1, bestZ2);

        telemetry.addData("Decision (locked)",
                chosenTagId == null ? "TAG3(default)" : (chosenTagId == TAG1_ID ? "TAG1" : "TAG2"));
        telemetry.update();

        // Post move (continue forward a bit to match your old behavior)
        // this is why it was driving forward again, after the first time
        /*
        Pose2d poseAfterPre = drive.pose;
        Action postMove = drive.actionBuilder(poseAfterPre)
                .lineToX(poseAfterPre.position.x + POSTMOVE_FORWARD_IN)
                .build();

        Actions.runBlocking(postMove);

        Action preMove = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(10.0, 0.0))   // go to x=10, y=0
                .build();

        Actions.runBlocking(preMove);




        // =========================
        // Execute chosen plan
        // =========================
        telemetry.addLine("Executing plan...");
        telemetry.addData("ChosenPlan", chosenTagId == null ? "TAG3 (default)" : (chosenTagId == TAG1_ID ? "TAG1" : "TAG2"));
        telemetry.update();

        if (chosenTagId == null) {
            doTag3Plan(drive);
        } else if (chosenTagId == TAG1_ID) {
            doTag1Plan(drive);
        } else if (chosenTagId == TAG2_ID) {
            doTag2Plan(drive);
        } else {
            doTag3Plan(drive);
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
     *  3) if tie and no pose: choose TAG2
     */
    /*
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

        return TAG2_ID;
    }

    // =========================
    // TAG PLANS (Road Runner movement)
    // =========================
    private void doTag1Plan(PinpointDrive drive) {
        telemetry.addLine("Running TAG 1 plan.");
        telemetry.update();

        // Replace your old timed turn + moveToPosition calls with RR actions.
        Pose2d start = drive.pose;

        Action tag1Move = drive.actionBuilder(start)
              //  .turn(TAG1_TURN_1_RAD)
                .strafeTo(TAG1_POINT_A)
                .strafeTo(TAG1_POINT_B)
                .build();

        Actions.runBlocking(tag1Move);
    }

    private void doTag2Plan(PinpointDrive drive) {
        telemetry.addLine("Running TAG 2 plan.");
        telemetry.update();

        shootingOrder2();



        Pose2d start = drive.pose;
        Action move = drive.actionBuilder(start)
                .strafeTo(new Vector2d(start.position.x, start.position.y + TAG23_STRAFE_LEFT_IN))
                .build();

        Actions.runBlocking(move);
    }

    private void doTag3Plan(PinpointDrive drive) {
        telemetry.addLine("Running TAG 3 DEFAULT plan.");
        telemetry.update();

        shootingOrder3();

        Pose2d start = drive.pose;
        Action move = drive.actionBuilder(start)
                .strafeTo(new Vector2d(start.position.x, start.position.y + TAG23_STRAFE_LEFT_IN))
                .build();

        Actions.runBlocking(move);
    }

    // =========================
    // Trap / shooter routines (unchanged)
    // =========================

*/
        /*
        private void runTrapServos500ms () {
            servoTrapLeft.setPower(-0.6);
            servoTrapRight.setPower(0.6);
            sleep(500);
            servoTrapLeft.setPower(0);
            servoTrapRight.setPower(0);
        }
            private void runTrapServos250s () {
            servoTrapLeft.setPower(-0.6);
            servoTrapRight.setPower(0.6);
            sleep(250);
            servoTrapLeft.setPower(0);
            servoTrapRight.setPower(0);
    }


        private void shooting () {
            shooterMotor.setVelocity(2000);
            try {
                body.run();// do other tasks while motor is on
                runTrapServos500ms;
                sleep(250);
                intakeMotor.setPower(-1.0);
                servoTrapLeft.setPower(-0.6);
                servoTrapRight.setPower(0.6);
                sleep(250);
                intakeMotor.setPower(0.0);
                runTrapServos500ms();
                sleep(250);

            } finally {
                shooterMotor.setVelocity(0);
            }
        }
/*
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
    // Shooter velocity helpers (unchanged)
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

     */
    }
}



