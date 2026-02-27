// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/OBTeleOp_Shooter_Velocity.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.oysterbay.base.RobotStructure;

@TeleOp(name = "This Is Driver Control", group = "OB")
public class OBTeleOp_Shooter extends OpMode {

    private final RobotStructure robot = new RobotStructure();

    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;
    private DcMotorEx intakeMotor;
    private DcMotorEx upperIntakeMotor;

    private Servo trapServo;

    // =========================
    // Shooter targets
    // =========================

    private static final double SHOOTER_TARGET_RPM = 3000.0;
    private static final double SHOOTER_REVERSE_RPM = -800.0;

    // =========================
    // Encoder conversion
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

    // Feed threshold
    private static final double FIRE_THRESHOLD = 0.92;

    @Override
    public void init() {
        robot.init(hardwareMap);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        upperIntakeMotor = hardwareMap.get(DcMotorEx.class, "upperIntakeMotor");

        trapServo = hardwareMap.get(Servo.class, "trapServo");

        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        upperIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotor.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        shooterMotor2.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0.0);

        upperIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperIntakeMotor.setPower(0.0);

        // Make sure shooter is stopped at init
        setShooterRpm(0.0);

        telemetry.addLine("TeleOp ready (simple shooter)");
        telemetry.addData("Shooter RPM Target", SHOOTER_TARGET_RPM);
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.driveFromGamepad(gamepad1, true, 0.8);



        // Shooter control (simple)
        // Right bumper = forward shooter RPM
        // D-pad up = reverse shooter RPM
        // Otherwise = stop
        double shooterTargetRpm = 0.0;

        if (gamepad1.dpad_up) {
            shooterTargetRpm = SHOOTER_REVERSE_RPM;
        } else if (gamepad1.right_bumper) {
            shooterTargetRpm = SHOOTER_TARGET_RPM;
        } else {
            shooterTargetRpm = 0.0;
        }

        setShooterRpm(shooterTargetRpm);

        // Upper intake / feed control
        double trig = Range.clip(gamepad1.right_trigger, 0.0, 1.0);

        if (gamepad1.dpad_down) {
            upperIntakeMotor.setPower(-0.2);
        } else if (!gamepad1.dpad_up && trig >= FIRE_THRESHOLD) {
            upperIntakeMotor.setPower(-0.2);
        } else if (gamepad1.left_bumper) {
            upperIntakeMotor.setPower(1.0);
            intakeMotor.setPower(-1.0);
        } else if (gamepad1.a) {
            intakeMotor.setPower(-1.0);
        } else {
            intakeMotor.setPower(0.0);
            upperIntakeMotor.setPower(0.0);
        }
        if (gamepad1.b) {
            trapServo.setPosition(0.51);
        }
        if(gamepad1.y){
            trapServo.setPosition(0.5);
        }

        // Telemetry
        double m1Rpm = Math.abs(ticksPerSecToRpm(shooterMotor.getVelocity()));
        double m2Rpm = Math.abs(ticksPerSecToRpm(shooterMotor2.getVelocity()));

        telemetry.addData("Shooter Target RPM", "%.0f", shooterTargetRpm);
        telemetry.addData("Motor1 RPM", "%.0f", m1Rpm);
        telemetry.addData("Motor2 RPM", "%.0f", m2Rpm);
        telemetry.update();
    }

    @Override
    public void stop() {
        setShooterRpm(0.0);

        if (intakeMotor != null) intakeMotor.setPower(0.0);
        if (upperIntakeMotor != null) upperIntakeMotor.setPower(0.0);

        robot.setDriverPowerZERO();
    }

    // =========================
    // Helpers
    // =========================
    private void setShooterRpm(double rpm) {
        double tps = rpmToTicksPerSec(rpm);
        shooterMotor.setVelocity(tps);
        shooterMotor2.setVelocity(tps);
    }

    private static double rpmToTicksPerSec(double rpm) {
        return (rpm * TICKS_PER_AXLE_REV) / 60.0;
    }

    private static double ticksPerSecToRpm(double tps) {
        return (tps * 60.0) / TICKS_PER_AXLE_REV;
    }
}
