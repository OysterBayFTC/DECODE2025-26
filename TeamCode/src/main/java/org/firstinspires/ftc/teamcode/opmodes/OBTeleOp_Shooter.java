// TeamCode/src/main/java/org/firstinspires/ftc/teamcode/oysterbay/teleop/OBTeleOp_Shooter.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.oysterbay.base.RobotStructure;

@TeleOp(name = "OB TeleOp + Shooter", group = "OB")
public class OBTeleOp_Shooter extends OpMode {

    private final RobotStructure robot = new RobotStructure();

    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private DcMotorEx intakeMotor;     // not used by trigger sequence, left available if you want manual control elsewhere
    private Servo tipperServo;

    private Servo leftHolderServo; // servo for sorting

    private Servo rightHolderServo; // servo for sorting


    // Shooter constants
    private static final double SHOOTER_POWER = 0.45;     // fixed spin power while trigger is in spin zone or fire zone
    private static final double SHOOTER_SCALE = 1.0;

    // Servo positions
    private static final double SERVO_FIRE_POS = 0.55;
    private static final double SERVO_REST_POS = 0.45;

    // Trigger thresholds
    private static final double SPIN_THRESHOLD = 0.08;    // slight press starts wheels
    private static final double FIRE_THRESHOLD = 0.92;    // full press fires the ball
    private static final double RESET_THRESHOLD = 0.40;   // must release below this before next fire cycle

    // Servo timing (milliseconds)
    private static final long FIRE_PULSE_MS = 220;        // time to hold at fire position

    // Intake default power if you decide to bump the ball forward during fire (optional, currently not used)
    private static final double INTAKE_SPEED = 0.30;

    // State
    private enum FireState { IDLE, SPINNING, FIRING, RECOVER }
    private FireState state = FireState.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean firedThisPress = false;  // prevents repeated fires while holding the trigger down

    @Override
    public void init() {
        // Drivetrain
        robot.init(hardwareMap);

        // Shooter mapping
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        tipperServo  = hardwareMap.get(Servo.class, "tipperServo");
        intakeMotor  = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        leftHolderServo  = hardwareMap.get(Servo.class, "leftHolderServo"); //in control hub slot #1
        rightHolderServo  = hardwareMap.get(Servo.class, "rightHolderServo"); // in control hub slot #2



        // Motor directions so positive power pulls the ball inward between wheels
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        tipperServo.setPosition(SERVO_REST_POS);
        leftHolderServo.setPosition(0.50);
        rightHolderServo.setPosition(0.50);

        telemetry.addLine("OB TeleOp + Shooter (Trigger Sequencer) ready");
        telemetry.addLine("Right trigger: slight press spins up, full press fires");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Normal drive on gamepad1; slow mode example kept as is
        boolean squaredInputs = true;
        double speedMult = gamepad1.left_bumper ? 0.55 : 1.0;
        robot.driveFromGamepad(gamepad1, squaredInputs, speedMult);

        // Trigger value on Logitech gamepad
        double trig = Range.clip(gamepad1.right_trigger, 0.0, 1.0);


        if (gamepad1.dpad_left) {
            leftHolderServo.setPosition(0.48);
        } else if (gamepad1.dpad_down) {
            leftHolderServo.setPosition(0.53);
        }
        if (gamepad1.dpad_right) {
            rightHolderServo.setPosition(0.5);
        } else if (gamepad1.dpad_up) {
            rightHolderServo.setPosition(0.55);
        }
        // button for intake
        if (gamepad1.a) {
            intakeMotor.setPower(-1.0);
        } else {
            intakeMotor.setPower(0.0);
        }
        // Determine zones for telemetry
        String zone;
        if (trig >= FIRE_THRESHOLD) zone = "FIRE ZONE";
        else if (trig >= SPIN_THRESHOLD) zone = "SPIN ZONE";
        else zone = "IDLE ZONE";

        // State machine
        switch (state) {
            case IDLE:
                // Wheels off, servo resting
                shooterLeft.setPower(0.0);
                shooterRight.setPower(0.0);
                tipperServo.setPosition(SERVO_REST_POS);
                firedThisPress = false;

                if (trig >= SPIN_THRESHOLD) {
                    // Start spinning wheels
                    setShooterPower(SHOOTER_POWER);
                    state = FireState.SPINNING;
                }
                break;

            case SPINNING:
                // Keep spinning while trigger held beyond spin threshold
                setShooterPower(SHOOTER_POWER);

                // Allow fire only once per full press
                if (trig >= FIRE_THRESHOLD && !firedThisPress) {
                    tipperServo.setPosition(SERVO_FIRE_POS);
                    timer.reset();
                    firedThisPress = true;
                    state = FireState.FIRING;
                } else if (trig < SPIN_THRESHOLD) {
                    // Released back to idle
                    state = FireState.IDLE;
                }
                break;

            case FIRING:
                // Hold servo at fire pos briefly, wheels keep spinning
                setShooterPower(SHOOTER_POWER);

                if (timer.milliseconds() >= FIRE_PULSE_MS) {
                    tipperServo.setPosition(SERVO_REST_POS);
                    state = FireState.RECOVER;
                }
                break;

            case RECOVER:
                // Wait for driver to release below a comfortable reset threshold to enable next shot
                setShooterPower(SHOOTER_POWER);

                if (trig < RESET_THRESHOLD) {
                    // Next cycle ready. If still held above spin threshold when they re-press, stay spinning.
                    if (trig >= SPIN_THRESHOLD) {
                        state = FireState.SPINNING;
                        firedThisPress = false;
                    } else {
                        state = FireState.IDLE;
                    }
                }
                break;
        }

        // Telemetry for the driver
        telemetry.addData("Trigger", "%.3f", trig);
        telemetry.addData("Zone", zone);
        telemetry.addData("Spin threshold", SPIN_THRESHOLD);
        telemetry.addData("Fire threshold", FIRE_THRESHOLD);
        telemetry.addData("Reset threshold", RESET_THRESHOLD);
        telemetry.addData("Shooter power", "%.2f", (shooterLeft.getPower() + shooterRight.getPower()) * 0.5);
        telemetry.addData("Servo", tipperServo.getPosition() >= (SERVO_FIRE_POS - 0.02) ? "FIRING" : "REST");
        telemetry.addData("State", state);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooterLeft != null)  shooterLeft.setPower(0);
        if (shooterRight != null) shooterRight.setPower(0);
        if (tipperServo != null)  tipperServo.setPosition(SERVO_REST_POS);
        if (intakeMotor != null)  intakeMotor.setPower(0);
        robot.setDriverPowerZERO();
    }

    private void setShooterPower(double pwr) {
        double power = Range.clip(pwr * SHOOTER_SCALE, -1.0, 1.0);
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }
}
