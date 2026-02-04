// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/autonomous/botAutonomousRR.java
package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "Auton13375_RED", group = "OB")
public class botAutonomous13375_RED extends LinearOpMode {

    // Road Runner units: inches + radians
    private static final Pose2d INITIAL_POSE = new Pose2d(0.0, 0.0, 0.0);

    private static final double CAM_LEFT_POS = 0.0;

    // Poses (tune)
    private static final Pose2d SHOOTING_POSITION = new Pose2d(-52.3, 0, Math.toRadians(0));

    private static final Pose2d TARGET_1 = new Pose2d(-44, -14.3, Math.toRadians(140));
    private static final Pose2d PICKUP_END_1 = new Pose2d(-23.3, -30.6, Math.toRadians(140));

    private static final Pose2d PICKUP_2 = new Pose2d(-64, -30.5, Math.toRadians(140));
    private static final Pose2d PICKUP_END_2 = new Pose2d(-32.9, -56, Math.toRadians(140));

    private static final Pose2d PICKUP_3 = new Pose2d(-76.7, -50.45, Math.toRadians(140));
    private static final Pose2d PICKUP_END_3 = new Pose2d(-49, -71, Math.toRadians(140));
    private static final Pose2d FINAL = new Pose2d(-42.4, 8.6, Math.toRadians(0));


    // =========================
    // Shooter VELOCITY (ticks/sec)
    // =========================
    // This is NOT RPM. It is encoder ticks per second as expected by DcMotorEx.setVelocity().
    // Tune this number on-robot.
    private static final double SHOOTER_TARGET_TICKS_PER_SEC = 2000.0;

    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;

    private CRServo servoTrapLeft;
    private CRServo servoTrapRight;


    @Override
    public void runOpMode() {

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        servoTrapLeft = hardwareMap.get(CRServo.class, "servoTrapLeft");
        servoTrapRight = hardwareMap.get(CRServo.class, "servoTrapRight");


        // You used camServo but never initialized it in your code

        // Shooter setup
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setShooterVelocityTicksPerSec(0.0);

        // Intake setup
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0.0);




        PinpointDrive drive = new PinpointDrive(hardwareMap, INITIAL_POSE);
        drive.pose = INITIAL_POSE;
        drive.updatePoseEstimate();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Pre-move
        runAction(drive, drive.actionBuilder(drive.pose)
                .lineToX(SHOOTING_POSITION.position.x)
                .build());
        sleep(100);
        shootSequenceTest();

        // Pickup 3
        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(PICKUP_2, Math.toRadians(0))
                .build());
        intakeMotor.setPower(-1.0);
        runAction(drive, drive.actionBuilder(drive.pose)
                .lineToX(PICKUP_END_2.position.x)
                .lineToY(PICKUP_END_2.position.y)
                .build());
        sleep(200);
        intakeMotor.setPower(0.0);

        // Back to shooting + shoot
        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(SHOOTING_POSITION, Math.toRadians(0))
                .build());
        movingBack();
        sleep(50);
        shootSequenceTest();

        // Move to target 1
        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(TARGET_1, Math.toRadians(0))
                .build());
        sleep(100);
        intakeMotor.setPower(-1.0);
        // End of pickup path 1
        runAction(drive, drive.actionBuilder(drive.pose)
                .lineToX(PICKUP_END_1.position.x)
                .lineToY(PICKUP_END_1.position.y)
                .build());
        sleep(200);
        intakeMotor.setPower(0);
        // Back to shooting + shoot
        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(SHOOTING_POSITION, Math.toRadians(0))
                .build());
        movingBack();
        sleep(50);
        shootSequenceTest();
        finalShots();
        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(FINAL, Math.toRadians(0))
                .build());

        // Stop everything
        setShooterVelocityTicksPerSec(0.0);
        intakeMotor.setPower(0.0);
        servoTrapLeft.setPower(0.0);
        servoTrapRight.setPower(0.0);
    }

    private void runAction(PinpointDrive drive, Action action) {
        Actions.runBlocking(action);
        drive.updatePoseEstimate();
    }

    // Velocity in ticks/sec (what DcMotorEx.setVelocity expects)
    private void setShooterVelocityTicksPerSec(double ticksPerSec) {
        shooterMotor.setVelocity(ticksPerSec);
    }

    private void runTrapServosMs(long ms) {
        servoTrapLeft.setPower(-0.6);
        servoTrapRight.setPower(0.6);
        sleep(ms);
        servoTrapLeft.setPower(0.0);
        servoTrapRight.setPower(0.0);
    }

    private void shootSequence() {
        // Spin up (ticks/sec)
        shooterMotor.setVelocity(2000);

        try {
            // Your original intent: trap servos, then feed with intake, etc.
            runTrapServosMs(500);
            sleep(250);

            intakeMotor.setPower(-1.0);
            runTrapServosMs(250);
            intakeMotor.setPower(0.0);

            sleep(250);
            runTrapServosMs(500);
            sleep(250);
        } finally {
            shooterMotor.setVelocity(0);
        }
    }
    private void shootSequenceTest() {
        // Spin up (ticks/sec)
        shooterMotor.setVelocity(1150);

        try {
            // Your original intent: trap servos, then feed with intake, etc.
            sleep(750);
            servoTrapLeft.setPower(-0.5);
            servoTrapRight.setPower(0.5);
            intakeMotor.setPower(-1.0);
            sleep(900);
            shooterMotor.setVelocity(800);
            servoTrapLeft.setPower(-0.5);
            servoTrapRight.setPower(0.5);
            intakeMotor.setPower(-1.0);
            sleep(600);
            servoTrapLeft.setPower(0);
            servoTrapRight.setPower(0);
            intakeMotor.setPower(0);
        } finally {
            shooterMotor.setVelocity(0);
        }
    }
    private void shootSequenceTestFirst() {
        // Spin up (ticks/sec)
        shooterMotor.setVelocity(1600);

        try {
            // Your original intent: trap servos, then feed with intake, etc.
            sleep(700);
            servoTrapLeft.setPower(-0.6);
            servoTrapRight.setPower(0.6);
            intakeMotor.setPower(-1.0);
            sleep(800);
            shooterMotor.setVelocity(950);
            servoTrapLeft.setPower(-0.6);
            servoTrapRight.setPower(0.6);
            intakeMotor.setPower(-1.0);
            sleep(500);


            servoTrapLeft.setPower(0);
            servoTrapRight.setPower(0);
            intakeMotor.setPower(0);
        } finally {
            shooterMotor.setVelocity(0);
        }
    }
    private void movingBack() {
        shooterMotor.setVelocity(-150);
        servoTrapLeft.setPower(0.6);
        servoTrapRight.setPower(-0.6);
        sleep(200);

    }
    private void finalShots() {
        // Spin up (ticks/sec)
        shooterMotor.setVelocity(2100);

        try {
            // Your original intent: trap servos, then feed with intake, etc.
            sleep(550);
            servoTrapLeft.setPower(-1.0);
            servoTrapRight.setPower(1.0);
            intakeMotor.setPower(-.2);
            sleep(800);
            shooterMotor.setVelocity(800);
            servoTrapLeft.setPower(-1.0);
            servoTrapRight.setPower(1.0);
            intakeMotor.setPower(-.2);
            sleep(500);


            servoTrapLeft.setPower(0);
            servoTrapRight.setPower(0);
            intakeMotor.setPower(0);
        } finally {
            shooterMotor.setVelocity(0);
        }
    }
}
