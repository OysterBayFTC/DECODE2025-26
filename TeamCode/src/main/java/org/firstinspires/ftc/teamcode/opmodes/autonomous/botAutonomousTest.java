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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "botAutonomousTest", group = "OB")
public class botAutonomousTest extends LinearOpMode {

    // Road Runner units: inches + radians
    private static final Pose2d INITIAL_POSE = new Pose2d(0.0, 0.0, 0.0);


    // Poses (tune)
    private static final Pose2d SHOOTING_POSITION = new Pose2d(-42, 0, Math.toRadians(0));

    private static final Pose2d SHOOTING_POS_FINAL = new Pose2d(-36.929, 13.709, Math.toRadians(-16));


    private static final Pose2d PICK_UP_1 = new Pose2d(-42.8, 21.4, Math.toRadians(-141));
    private static final Pose2d PICKUP_END_1 = new Pose2d(-26.5, 36.6, Math.toRadians(-141));

    private static final Pose2d PICKUP_2 = new Pose2d(-57.76, 38.8, Math.toRadians(-141));
    private static final Pose2d PICKUP_END_2 = new Pose2d(-37.5, 56.3, Math.toRadians(-141));

    private static final Pose2d PICKUP_3 = new Pose2d(-71.3, 57, Math.toRadians(-141));
    private static final Pose2d PICKUP_END_3 = new Pose2d(-53.5, 73.8, Math.toRadians(-141));

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
                .lineToX(-35)
                .build());

        shootSequenceTestFirst();

        // Pickup 3
        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(PICKUP_3, Math.toRadians(0))
                .build());
        intakeMotor.setPower(-1.0);
        runAction(drive, drive.actionBuilder(drive.pose)
                .lineToX(PICKUP_END_3.position.x)
                .lineToY(PICKUP_END_3.position.y)
                .build());
        intakeMotor.setPower(0.0);

        // Back to shooting + shoot
        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(SHOOTING_POSITION, Math.toRadians(0))
                .build());
        movingBack();
        sleep(100);
        shootSequenceTestFirst();

        // Pickup 2
        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(PICKUP_2, Math.toRadians(0))
                .build());
        intakeMotor.setPower(-1.0);
        runAction(drive, drive.actionBuilder(drive.pose)
                .lineToX(PICKUP_END_2.position.x)
                .lineToY(PICKUP_END_2.position.y)
                .build());
        intakeMotor.setPower(0.0);

        // Back to shooting + shoot
        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(SHOOTING_POSITION, Math.toRadians(0))
                .build());
        movingBack();
        sleep(100);
        shootSequenceTestFirst();

        // Move to target 1
        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(PICK_UP_1, Math.toRadians(0))
                .build());
        intakeMotor.setPower(-1.0);
        // End of pickup path 1
        runAction(drive, drive.actionBuilder(drive.pose)
                .lineToX(PICKUP_END_1.position.x)
                .lineToY(PICKUP_END_1.position.y)
                .build());
        intakeMotor.setPower(0);
        // Back to shooting + shoot
        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(SHOOTING_POS_FINAL, Math.toRadians(0))
                .build());
        movingBack();
        sleep(100);
        shootSequenceTestFirst();



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
            sleep(600);
            servoTrapLeft.setPower(-0.6);
            servoTrapRight.setPower(0.6);
            intakeMotor.setPower(-1.0);
            sleep(800);
            shooterMotor.setVelocity(850);
            servoTrapLeft.setPower(-0.6);
            servoTrapRight.setPower(0.6);
            intakeMotor.setPower(-1.0);
            sleep(400);
            servoTrapLeft.setPower(0);
            servoTrapRight.setPower(0);
            intakeMotor.setPower(0);
        } finally {
            shooterMotor.setVelocity(0);
        }
    }
    private void shootSequenceTestFirst() {
        // Spin up (ticks/sec)
        shooterMotor.setVelocity(2300);

        try {
            // Your original intent: trap servos, then feed with intake, etc.
            sleep(550);
            servoTrapLeft.setPower(-1.0);
            servoTrapRight.setPower(1.0);
            intakeMotor.setPower(-1.0);
            sleep(800);
            shooterMotor.setVelocity(1000);
            servoTrapLeft.setPower(-1.0);
            servoTrapRight.setPower(1.0);
            intakeMotor.setPower(-1.0);
            sleep(300);


            servoTrapLeft.setPower(0);
            servoTrapRight.setPower(0);
            intakeMotor.setPower(0);
        } finally {
            shooterMotor.setVelocity(0);
        }
    }
    private void movingBack() {
        shooterMotor.setVelocity(-200);
        servoTrapLeft.setPower(0.6);
        servoTrapRight.setPower(-0.6);
        sleep(200);

    }
}
