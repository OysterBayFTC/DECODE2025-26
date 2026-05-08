// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/autonomous/botAutonomousRR.java
// this code only pick up the first 2 rows of ball, due partner
package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "BlueBackPickUp", group = "OB")
public class BlueBackPickUp extends LinearOpMode {

    // Road Runner units: inches + radians
    private static final Pose2d INITIAL_POSE = new Pose2d(0.0, 0.0, 0.0);

    // Poses (tune)
    private static final Pose2d SHOOTING_POSITION  = new Pose2d(5.4, 1.4, Math.toRadians(15.91));
    private static final Pose2d PARKING_POSITION  = new Pose2d(8.9, 16.5, Math.toRadians(0));

    private static final Pose2d SHOOTING_POS_FINAL = new Pose2d(-36.929, 13.709, Math.toRadians(-18));

    private static final Pose2d PICK_UP_1    = new Pose2d(-43.6, -17.0, Math.toRadians(141));
    private static final Pose2d PICKUP_END_1 = new Pose2d(-26.5, -36.6, Math.toRadians(141));

    private static final Pose2d PICKUP_2     = new Pose2d(-59.76, -40.8, Math.toRadians(141));
    private static final Pose2d PICKUP_END_2 = new Pose2d(-37.5, -56.3, Math.toRadians(141));

    private static final Pose2d PICKUP_3     = new Pose2d(26.6, 8.0, Math.toRadians(-90));
    private static final Pose2d PICKUP_END_3 = new Pose2d(26.6, 48.8, Math.toRadians(-90));

    // =========================
    // Shooter targets (RPM)  -> converted to ticks/sec for setVelocity()
    // =========================
    private static final double SHOOTER_TARGET_RPM = 3730.0;

    // =========================
    // Encoder conversion (match your TeleOp)
    // =========================
    private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0;
    private static final double GEAR_RATIO_MOTOR_TO_AXLE = 1.0; // change if needed
    private static final double TICKS_PER_AXLE_REV = ENCODER_TICKS_PER_MOTOR_REV * GEAR_RATIO_MOTOR_TO_AXLE;

    // =========================
    // Velocity PIDF
    // =========================
    private static final double SHOOTER_kP = 5.0;
    private static final double SHOOTER_kI = 0.0;
    private static final double SHOOTER_kD = 0.0;
    private static final double SHOOTER_kF = 12.6;

    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;

    private DcMotorEx intakeMotor;
    private DcMotorEx upperIntakeMotor;

    private CRServo servoTrapLeft;
    private CRServo servoTrapRight;
    private Servo trapServo;

    @Override
    public void runOpMode() {

        shooterMotor  = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2"); // FIXED

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        upperIntakeMotor = hardwareMap.get(DcMotorEx.class, "upperIntakeMotor");

        servoTrapLeft = hardwareMap.get(CRServo.class, "servoTrapLeft");
        servoTrapRight = hardwareMap.get(CRServo.class, "servoTrapRight");
        trapServo = hardwareMap.get(Servo.class, "trapServo");

        // Shooter setup
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMPORTANT: Match TeleOp directions
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        shooterMotor2.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        setShooterRpm(0.0);

        // Intake setup
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0.0);

        upperIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        upperIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperIntakeMotor.setPower(0.0);

        PinpointDrive drive = new PinpointDrive(hardwareMap, INITIAL_POSE);
        drive.pose = INITIAL_POSE;
        drive.updatePoseEstimate();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Pre-move + first shots
        setShooterRpm(SHOOTER_TARGET_RPM);

        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(SHOOTING_POSITION, Math.toRadians(0))
                .build());

        trapServo.setPosition(0.82);
        sleep(1000);
        shootSequence();
        sleep(500);
        trapServo.setPosition(0.15);

        // Pickup 1
        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(PICKUP_3, Math.toRadians(0))
                .build());

        intakeMotor.setPower(-1.0);
        runAction(drive, drive.actionBuilder(drive.pose)
                .lineToX(PICKUP_END_3.position.x)
                .lineToY(PICKUP_END_3.position.y)
                .build());
        intakeMotor.setPower(0.0);

        trapServo.setPosition(0.05);
        sleep(200);

        // Back to shooting + shoot
        setShooterRpm(SHOOTER_TARGET_RPM);
        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(SHOOTING_POSITION, Math.toRadians(0))
                .build());

        trapServo.setPosition(0.82);
        sleep(1000);
        shootSequence();
        sleep(500);
        trapServo.setPosition(0.15);

        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(PARKING_POSITION, Math.toRadians(0))
                .build());
        setShooterRpm(0);
/*
        trapServo.setPosition(0.82);
        sleep(1000);
        shootSequence();
        trapServo.setPosition(0.15);

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

        trapServo.setPosition(0.05);
        sleep(200);

        // Back to shooting + shoot
        setShooterRpm(SHOOTER_TARGET_RPM);
        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(SHOOTING_POSITION, Math.toRadians(0))
                .build());
        movingBack();

        trapServo.setPosition(0.82);
        sleep(1000);
        shootSequence();
        trapServo.setPosition(0.15);
/*
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
        setShooterRpm(SHOOTER_TARGET_RPM);
        runAction(drive, drive.actionBuilder(drive.pose)
                .splineToLinearHeading(SHOOTING_POSITION, Math.toRadians(0))
                .build());
        movingBack();

        trapServo.setPosition(0.82);
        sleep(1000);
        shootSequence();
*/
        // Stop everything
        setShooterRpm(0.0);
        intakeMotor.setPower(0.0);
        upperIntakeMotor.setPower(0.0);
        servoTrapLeft.setPower(0.0);
        servoTrapRight.setPower(0.0);
    }

    /**
     * Runs a Road Runner Action while continuously updating pose + printing X/Y/Heading.
     * Telemetry updates during motion.
     */
    private void runAction(PinpointDrive drive, Action action) {
        TelemetryPacket packet = new TelemetryPacket();

        while (opModeIsActive() && action.run(packet)) {
            drive.updatePoseEstimate();

            Pose2d p = drive.pose;
            telemetry.addData("X (in)", "%.2f", p.position.x);
            telemetry.addData("Y (in)", "%.2f", p.position.y);
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(p.heading.toDouble()));
            telemetry.update();

            idle();
            packet = new TelemetryPacket();
        }

        drive.updatePoseEstimate();
        Pose2d p = drive.pose;
        telemetry.addData("X (in)", "%.2f", p.position.x);
        telemetry.addData("Y (in)", "%.2f", p.position.y);
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(p.heading.toDouble()));
        telemetry.update();
    }

    // =========================
    // Shooter helpers (RPM -> ticks/sec)
    // =========================
    private void setShooterRpm(double rpm) {
        double tps = rpmToTicksPerSec(rpm);
        shooterMotor.setVelocity(tps);
        shooterMotor2.setVelocity(tps);
    }

    private static double rpmToTicksPerSec(double rpm) {
        return (rpm * TICKS_PER_AXLE_REV) / 60.0;
    }

    @SuppressWarnings("unused")
    private static double ticksPerSecToRpm(double tps) {
        return (tps * 60.0) / TICKS_PER_AXLE_REV;
    }

    // =========================
    // Shooting / feeding
    // =========================
    private void shootSequence() {
        // Assumes shooter is already spun up (setShooterRpm called before this)
        intakeMotor.setPower(-1.0);
        upperIntakeMotor.setPower(1.0);
        sleep(100);
        intakeMotor.setPower(0.0);
        upperIntakeMotor.setPower(0.0);
        sleep(500);

        intakeMotor.setPower(-1.0);
        upperIntakeMotor.setPower(1.0);
        sleep(100);
        intakeMotor.setPower(0.0);
        upperIntakeMotor.setPower(0.0);
        sleep(500);

        intakeMotor.setPower(-1.0);
        upperIntakeMotor.setPower(1.0);
        sleep(100);
        intakeMotor.setPower(0.0);
        upperIntakeMotor.setPower(0.0);
        sleep(500);

        intakeMotor.setPower(-1.0);
        upperIntakeMotor.setPower(1.0);
        sleep(100);
        intakeMotor.setPower(0.0);
        upperIntakeMotor.setPower(0.0);
        sleep(500);

        intakeMotor.setPower(-1.0);
        upperIntakeMotor.setPower(1.0);
        sleep(100);
        intakeMotor.setPower(0.0);
        upperIntakeMotor.setPower(0.0);
        sleep(500);

        intakeMotor.setPower(-1.0);
        upperIntakeMotor.setPower(1.0);
        sleep(100);
        intakeMotor.setPower(0.0);
        upperIntakeMotor.setPower(0.0);
        sleep(500);
    }

    private void movingBack() {
        // Do NOT reverse shooter here (that can spit backwards).
        // Just run your trap servos.
        servoTrapLeft.setPower(0.6);
        servoTrapRight.setPower(-0.6);
        sleep(200);
        servoTrapLeft.setPower(0.0);
        servoTrapRight.setPower(0.0);
    }
}
