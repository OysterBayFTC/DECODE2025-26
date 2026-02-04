// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/tuning/ShooterPIDF_A_Toggle.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Testing PIDF", group = "OB")
public class Testing_PIDF extends LinearOpMode {

    // Keep your same variable name
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;

    private CRServo servoTrapLeft;
    private CRServo servoTrapRight;

    // =========================
    // EDIT THESE TO TEST PIDF
    // =========================
    private static final double kP = 60.0;
    private static final double kI = 0.0;
    private static final double kD = 8.0;
    private static final double kF = 13.0;

    // Target velocity (ticks/sec)
    private static final double SHOOTER_TARGET_TICKS_PER_SEC = 2300.0;

    // Simple protection so you don’t accidentally slam full power on a bad tune
    private static final double MAX_TARGET_TICKS_PER_SEC = 4000.0;

    @Override
    public void runOpMode() {

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        servoTrapLeft = hardwareMap.get(CRServo.class, "servoTrapLeft");
        servoTrapRight = hardwareMap.get(CRServo.class, "servoTrapRight");



        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0.0);

        // Apply your test PIDF
        shooterMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        telemetry.addLine("Press A to toggle shooter velocity ON/OFF.");
        telemetry.addData("PIDF", "P=%.3f I=%.3f D=%.3f F=%.3f", kP, kI, kD, kF);
        telemetry.addData("Target (ticks/sec)", SHOOTER_TARGET_TICKS_PER_SEC);
        telemetry.addLine("Edit kP/kI/kD/kF in code, redeploy, repeat.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        boolean shooterOn = false;
        boolean lastA = false;

        while (opModeIsActive()) {
            boolean a = gamepad1.a;

            // Toggle on rising edge
            if (a && !lastA) {
                shooterOn = !shooterOn;
            }
            lastA = a;

            double target = SHOOTER_TARGET_TICKS_PER_SEC;
            if (target > MAX_TARGET_TICKS_PER_SEC) target = MAX_TARGET_TICKS_PER_SEC;
            if (target < 0) target = 0;

            if (shooterOn) {
                shooterMotor.setVelocity(target);
                servoTrapLeft.setPower(-0.6);
                servoTrapRight.setPower(0.6);
                intakeMotor.setPower(-1.0);
            } else {
                shooterMotor.setVelocity(0.0);
            }

            double v = shooterMotor.getVelocity();

            telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("Target (ticks/sec)", target);
            telemetry.addData("Velocity (ticks/sec)", v);
            telemetry.addData("Error", (target - v));
            telemetry.update();

            sleep(20);
        }

        shooterMotor.setVelocity(0.0);
    }
}
