// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/OBTeleOp_Shooter_Velocity.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.oysterbay.base.RobotStructure;

@TeleOp(name = "This Is Driver Control", group = "OB")
public class OBTeleOp_Shooter extends OpMode {

    private final RobotStructure robot = new RobotStructure();

    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private CRServo servoTrapLeft;
    private CRServo servoTrapRight;

    // =========================
    // Shooter constants
    // =========================
    private static final double SHOOTER_TICKS_PER_REV = 28.0;

    private static final double SHOOTER_TARGET_RPM = 2750;
    private static final double SHOOTER_REVERSE_RPM = -800.0; // DPAD_UP

    // Trigger thresholds
    private static final double SPIN_THRESHOLD = 0.08;   // start spinning shooter
    private static final double FIRE_THRESHOLD = 0.92;   // "fully depressed" -> run trap feed

    // Trap powers
    private static final double TRAP_B_POWER = 0.6;        // your B behavior magnitude
    private static final double TRAP_FIRE_POWER = 0.45;    // slightly slower than B (tune)

    @Override
    public void init() {
        robot.init(hardwareMap);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        intakeMotor  = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        servoTrapLeft  = hardwareMap.get(CRServo.class, "servoTrapLeft");
        servoTrapRight = hardwareMap.get(CRServo.class, "servoTrapRight");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setShooterRpm(0.0);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0.0);

        servoTrapLeft.setPower(0);
        servoTrapRight.setPower(0);

        telemetry.addLine("TeleOp ready");
        telemetry.addLine("Trigger: spin shooter, full trigger: trap feed slower");
        telemetry.addLine("DPAD_UP: reverse shooter @ 400rpm");
        telemetry.update();
    }

    @Override
    public void loop() {
        // =========================
        // Drive
        // =========================
        robot.driveFromGamepad(gamepad1, true, 0.8);

        // =========================
        // Intake
        // =========================
        if (gamepad1.a) {
            intakeMotor.setPower(-1.0);
        } else {
            intakeMotor.setPower(0.0);
        }

        // Read trigger (use RIGHT trigger as the single shooter control)
        double trig = Range.clip(gamepad1.right_trigger, 0.0, 1.0);

        // =========================
        // Shooter control
        // DPAD_UP overrides trigger and runs shooter backwards slowly
        // =========================
        if (gamepad1.dpad_up) {
            setShooterRpm(SHOOTER_REVERSE_RPM);
        } else {
            if (gamepad1.right_bumper) {
                setShooterRpm(SHOOTER_TARGET_RPM);
            } else {
                setShooterRpm(0.0);
            }
        }
        if (gamepad1.left_bumper) {
            servoTrapLeft.setPower(-TRAP_B_POWER);
            servoTrapRight.setPower(TRAP_B_POWER);
            if (!gamepad1.left_bumper) {
                servoTrapLeft.setPower(0);
                servoTrapRight.setPower(0);

            }
        }

        // =========================
        // Trap control
        // Priority: DPAD_DOWN, then full trigger feed, then B, else stop
        // =========================
        if (gamepad1.dpad_down) {
            // Your "down" behavior
            servoTrapLeft.setPower( TRAP_B_POWER);
            servoTrapRight.setPower(-TRAP_B_POWER);
        } else if (!gamepad1.dpad_up && trig >= FIRE_THRESHOLD) {
            // Full trigger feed: same direction as B behavior, but slower
            servoTrapLeft.setPower(-TRAP_FIRE_POWER);
            servoTrapRight.setPower( TRAP_FIRE_POWER);
        } else if (gamepad1.b) {
            // Your "B" behavior

        } else {
            servoTrapLeft.setPower(0);
            servoTrapRight.setPower(0);
        }


        // =========================
        // Telemetry
        // =========================
        telemetry.addData("Trigger", "%.2f", trig);
        telemetry.addData("Shooter RPM", "%.0f", ticksPerSecToRpm(shooterMotor.getVelocity()));
        telemetry.addData("Shooter Cmd", gamepad1.dpad_up ? "REV 400" : (trig >= SPIN_THRESHOLD ? "2000" : "0"));
        telemetry.addData("Trap Cmd",
                gamepad1.dpad_down ? "DPAD_DOWN" :
                        (!gamepad1.dpad_up && trig >= FIRE_THRESHOLD) ? "TRIG_FEED_SLOW" :
                                (gamepad1.b ? "B" : "STOP"));
        telemetry.update();
    }

    @Override
    public void stop() {
        setShooterRpm(0.0);
        if (intakeMotor != null) intakeMotor.setPower(0.0);
        if (servoTrapLeft != null) servoTrapLeft.setPower(0);
        if (servoTrapRight != null) servoTrapRight.setPower(0);
        robot.setDriverPowerZERO();
    }

    // =========================
    // Shooter helpers
    // =========================
    private void setShooterRpm(double rpm) {
        shooterMotor.setVelocity(rpmToTicksPerSec(rpm));
    }

    private static double rpmToTicksPerSec(double rpm) {
        return (rpm * SHOOTER_TICKS_PER_REV) / 60.0;
    }

    private static double ticksPerSecToRpm(double tps) {
        return (tps * 60.0) / SHOOTER_TICKS_PER_REV;
    }
    // File: ShooterController.java
// Use with DcMotorEx shooterMotor





}
