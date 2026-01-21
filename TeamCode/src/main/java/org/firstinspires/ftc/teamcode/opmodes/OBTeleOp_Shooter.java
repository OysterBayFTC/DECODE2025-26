// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/OBTeleOp_Shooter_Velocity.java
package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.oysterbay.base.RobotStructure;

/**
 * TeleOp that matches your original driver controls, but uses setVelocity() for the shooter.
 *
 * Updates you requested:
 * 1) Driving flipped front/back (done in RobotStructure)
 * 2) Holder servo controls remapped:
 *    - DPAD_LEFT opens left holder
 *    - DPAD_RIGHT opens right holder
 *    - DPAD_UP opens BOTH
 *    - DPAD_DOWN closes BOTH
 * 3) Turning speed +20% (done in RobotStructure)
 * 4) LEFT trigger = short shot at 1800 RPM with same logic as right trigger
 */
@TeleOp(name = "OB TeleOp + Shooter (Velocity)", group = "OB")
public class OBTeleOp_Shooter extends OpMode {

    private final RobotStructure robot = new RobotStructure();

    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private DcMotorEx intakeMotor;

    private Servo tipperServo;

    private CRServo servoTrapLeft;
    private CRServo servoTrapRight;

    private Servo leftHolderServo;
    private Servo rightHolderServo;

    // =========================
    // Camera (Driver Station live view)
    // =========================
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProc;

    private static final String WEBCAM_NAME = "Webcam 1";

    // =========================
    // Shooter Velocity Constants
    // =========================
    private static final double SHOOTER_TICKS_PER_REV = 28.0;

    // Long vs Short RPM
    private static final double SHOOTER_LONG_TARGET_RPM  = 1950.0; // right trigger
    private static final double SHOOTER_SHORT_TARGET_RPM = 1800.0; // left trigger

    // Scale multiplier if you ever want to tweak easily
    private static final double SHOOTER_SCALE = 1.0;

    // Allow fire only when within tolerance
    private static final boolean REQUIRE_AT_SPEED_TO_FIRE = true;
    private static final double AT_SPEED_TOL_RPM = 150.0;
    private static final double SPINUP_TIMEOUT_SEC = 1.5;

    // Command signs for inward spinning wheels
    private static final int LEFT_CMD_SIGN = +1;
    private static final int RIGHT_CMD_SIGN = -1;
    private static final double LEFT_TRIM  = 1.00;
    private static final double RIGHT_TRIM = 1.00;

    // =========================
    // Servo positions
    // =========================
    private static final double SERVO_FIRE_POS = 0.25;
    private static final double SERVO_REST_POS = 0.10;

    // Holder servo positions (your existing values)
    private static final double LEFT_HOLDER_OPEN  = 0.5;
    private static final double LEFT_HOLDER_CLOSE = 0.15;

    private static final double RIGHT_HOLDER_OPEN  = 0.00;
    private static final double RIGHT_HOLDER_CLOSE = 0.40;

    // Trigger thresholds
    private static final double SPIN_THRESHOLD = 0.08;
    private static final double FIRE_THRESHOLD = 0.92;
    private static final double RESET_THRESHOLD = 0.40;

    // Servo timing
    private static final long FIRE_PULSE_MS = 220;

    // State machine
    private enum FireState { IDLE, SPINNING, FIRING, RECOVER }
    private FireState state = FireState.IDLE;

    private enum ShotMode { NONE, SHORT, LONG }
    private ShotMode shotMode = ShotMode.NONE;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime spinupTimer = new ElapsedTime();
    private boolean firedThisPress = false;

    @Override
    public void init() {
        // Drivetrain
        robot.init(hardwareMap);

        // Shooter mapping
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        tipperServo  = hardwareMap.get(Servo.class, "tipperServo");
        intakeMotor  = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        leftHolderServo  = hardwareMap.get(Servo.class, "leftHolderServo");
        rightHolderServo = hardwareMap.get(Servo.class, "rightHolderServo");

        servoTrapLeft  = hardwareMap.get(CRServo.class, "servoTrapLeft");
        servoTrapRight = hardwareMap.get(CRServo.class, "servoTrapRight");

        // Shooter motor setup for setVelocity()
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Intake
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servo init
        tipperServo.setPosition(SERVO_REST_POS);

        leftHolderServo.setPosition(LEFT_HOLDER_CLOSE);
        rightHolderServo.setPosition(RIGHT_HOLDER_CLOSE);

        // Stop shooter explicitly
        setShooterRpm(0.0);

        // Camera init (with explicit camera selected)
        tagProc = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(tagProc)
                .enableLiveView(true)
                .build();

        telemetry.addLine("OB TeleOp + Shooter (Velocity) ready");
        telemetry.addLine("Right trigger = long shot, Left trigger = short shot");
        telemetry.addData("Long RPM", "%.0f", SHOOTER_LONG_TARGET_RPM);
        telemetry.addData("Short RPM", "%.0f", SHOOTER_SHORT_TARGET_RPM);
        telemetry.update();
    }

    @Override
    public void loop() {
        // =========================
        // Normal drive
        // =========================
        boolean squaredInputs = true;
        double speedMult = 0.8;
        if (gamepad1.x) {
            speedMult = 1.5;
            if (!gamepad1.x) {
                speedMult = 0.8;
            }
        } else {
            speedMult = 0.8;
        }
        robot.driveFromGamepad(gamepad1, squaredInputs, speedMult);

        // =========================
        // Trap CR servos
        // =========================
        if (gamepad1.b) {
            servoTrapLeft.setPower(-0.6);
            servoTrapRight.setPower(0.6);
        } else {
            servoTrapLeft.setPower(0);
            servoTrapRight.setPower(0);
        }

        // =========================
        // Holder servos (NEW MAPPING)
        // =========================
        if (gamepad1.dpad_left) {
            leftHolderServo.setPosition(LEFT_HOLDER_OPEN);
        }
        if (gamepad1.dpad_right) {
            rightHolderServo.setPosition(RIGHT_HOLDER_OPEN);
        }
        if (gamepad1.dpad_up) {
            leftHolderServo.setPosition(LEFT_HOLDER_OPEN);
            rightHolderServo.setPosition(RIGHT_HOLDER_OPEN);
        }
        if (gamepad1.dpad_down) {
            leftHolderServo.setPosition(LEFT_HOLDER_CLOSE);
            rightHolderServo.setPosition(RIGHT_HOLDER_CLOSE);
        }

        // =========================
        // Intake
        // =========================
        if (gamepad1.a) {
            intakeMotor.setPower(-1.0);
        } else {
            intakeMotor.setPower(0.0);
        }

        // =========================
        // Your existing Y behavior (kept)
        // =========================
        if (gamepad1.y) {
            setShooterRpm(-900.0);

            servoTrapLeft.setPower(.20);
            servoTrapRight.setPower(-0.20);

            intakeMotor.setPower(1.0);
            telemetry.addData("OVERRIDE", "Y held (reverse clear)");
            telemetry.update();
            return;
        }

        // =========================
        // Dual-trigger selection
        // Right trigger = LONG, Left trigger = SHORT
        // Priority: LONG if both pressed
        // =========================
        double trigLong  = Range.clip(gamepad1.right_trigger, 0.0, 1.0);
        double trigShort = Range.clip(gamepad1.left_trigger,  0.0, 1.0);

        ShotMode requestedMode = ShotMode.NONE;
        double activeTrig = 0.0;
        double activeTargetRpm = 0.0;

        if (trigLong >= SPIN_THRESHOLD) {
            requestedMode = ShotMode.LONG;
            activeTrig = trigLong;
            activeTargetRpm = SHOOTER_LONG_TARGET_RPM;
        } else if (trigShort >= SPIN_THRESHOLD) {
            requestedMode = ShotMode.SHORT;
            activeTrig = trigShort;
            activeTargetRpm = SHOOTER_SHORT_TARGET_RPM;
        }

        // =========================
        // State machine (shared for both triggers)
        // =========================
        switch (state) {
            case IDLE:
                setShooterRpm(0.0);
                tipperServo.setPosition(SERVO_REST_POS);
                firedThisPress = false;
                shotMode = ShotMode.NONE;

                if (requestedMode != ShotMode.NONE) {
                    shotMode = requestedMode;
                    setShooterRpm(activeTargetRpm);
                    spinupTimer.reset();
                    state = FireState.SPINNING;
                }
                break;

            case SPINNING:
                // If neither trigger is held anymore, stop
                if (requestedMode == ShotMode.NONE) {
                    state = FireState.IDLE;
                    break;
                }

                // If driver switches modes while spinning, swap targets smoothly
                shotMode = requestedMode;
                activeTargetRpm = (shotMode == ShotMode.LONG) ? SHOOTER_LONG_TARGET_RPM : SHOOTER_SHORT_TARGET_RPM;
                activeTrig = (shotMode == ShotMode.LONG) ? trigLong : trigShort;

                setShooterRpm(activeTargetRpm);

                boolean atSpeed = shooterAtSpeed(activeTargetRpm, AT_SPEED_TOL_RPM);
                boolean timedOut = spinupTimer.seconds() >= SPINUP_TIMEOUT_SEC;

                if (activeTrig >= FIRE_THRESHOLD && !firedThisPress) {
                    if (!REQUIRE_AT_SPEED_TO_FIRE || atSpeed || timedOut) {
                        tipperServo.setPosition(SERVO_FIRE_POS);
                        timer.reset();
                        firedThisPress = true;
                        state = FireState.FIRING;
                    }
                }
                break;

            case FIRING:
                // Keep current mode target RPM while firing
                activeTargetRpm = (shotMode == ShotMode.LONG) ? SHOOTER_LONG_TARGET_RPM : SHOOTER_SHORT_TARGET_RPM;
                setShooterRpm(activeTargetRpm);

                if (timer.milliseconds() >= FIRE_PULSE_MS) {
                    tipperServo.setPosition(SERVO_REST_POS);
                    state = FireState.RECOVER;
                }
                break;

            case RECOVER:
                // Keep current mode target RPM while recovering
                activeTargetRpm = (shotMode == ShotMode.LONG) ? SHOOTER_LONG_TARGET_RPM : SHOOTER_SHORT_TARGET_RPM;
                setShooterRpm(activeTargetRpm);

                // Determine which trigger is active for reset logic
                activeTrig = (shotMode == ShotMode.LONG) ? trigLong : trigShort;

                if (activeTrig < RESET_THRESHOLD) {
                    firedThisPress = false;

                    // If they are still lightly holding either trigger, go back to SPINNING
                    if (requestedMode != ShotMode.NONE) {
                        shotMode = requestedMode;
                        spinupTimer.reset();
                        state = FireState.SPINNING;
                    } else {
                        state = FireState.IDLE;
                    }
                }
                break;
        }

        // =========================
        // Telemetry
        // =========================
        telemetry.addData("Camera", visionPortal == null ? "null" : visionPortal.getCameraState());
        telemetry.addData("Tag detections", (tagProc == null || tagProc.getDetections() == null) ? 0 : tagProc.getDetections().size());

        telemetry.addData("Mode", shotMode);
        telemetry.addData("Long Trigger", "%.3f", trigLong);
        telemetry.addData("Short Trigger", "%.3f", trigShort);

        telemetry.addData("Shooter L pos", shooterLeft.getCurrentPosition());
        telemetry.addData("Shooter R pos", shooterRight.getCurrentPosition());

        telemetry.addData("Shooter Target RPM", "%.0f",
                (shotMode == ShotMode.LONG) ? SHOOTER_LONG_TARGET_RPM :
                        (shotMode == ShotMode.SHORT) ? SHOOTER_SHORT_TARGET_RPM : 0.0);

        telemetry.addData("Shooter L RPM (shooting+)", "%.0f", getShooterRpmShootPositive(shooterLeft, LEFT_CMD_SIGN));
        telemetry.addData("Shooter R RPM (shooting+)", "%.0f", getShooterRpmShootPositive(shooterRight, RIGHT_CMD_SIGN));

        double displayTarget = (shotMode == ShotMode.LONG) ? SHOOTER_LONG_TARGET_RPM :
                (shotMode == ShotMode.SHORT) ? SHOOTER_SHORT_TARGET_RPM : 0.0;

        telemetry.addData("At speed?", (shotMode == ShotMode.NONE) ? false : shooterAtSpeed(displayTarget, AT_SPEED_TOL_RPM));
        telemetry.addData("State", state);
        telemetry.addData("Tipper", tipperServo.getPosition() >= (SERVO_FIRE_POS - 0.02) ? "FIRING" : "REST");

        telemetry.update();
    }

    @Override
    public void stop() {
        setShooterRpm(0.0);

        if (intakeMotor != null) intakeMotor.setPower(0.0);
        if (tipperServo != null) tipperServo.setPosition(SERVO_REST_POS);

        servoTrapLeft.setPower(0);
        servoTrapRight.setPower(0);

        robot.setDriverPowerZERO();

        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }

    // =========================
    // Shooter helpers
    // =========================
    private void setShooterRpm(double rpm) {
        double scaledRpm = rpm * SHOOTER_SCALE;
        double tps = rpmToTicksPerSec(scaledRpm);

        shooterLeft.setVelocity(LEFT_CMD_SIGN * tps * LEFT_TRIM);
        shooterRight.setVelocity(RIGHT_CMD_SIGN * tps * RIGHT_TRIM);
    }

    private boolean shooterAtSpeed(double targetRpm, double tolRpm) {
        double l = getShooterRpmShootPositive(shooterLeft, LEFT_CMD_SIGN);
        double r = getShooterRpmShootPositive(shooterRight, RIGHT_CMD_SIGN);

        double tgt = Math.abs(targetRpm);
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
}
