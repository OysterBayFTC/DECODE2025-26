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
 * Key change:
 *  - Both shooter motors are set to Direction.FORWARD
 *  - We explicitly command the RIGHT shooter motor with a NEGATIVE velocity so the wheels spin inward.
 *  - Telemetry is "sign-corrected" so BOTH RPMs show positive when shooting.
 *
 * This avoids "runaway" behavior and fixes the "right encoder goes negative" issue (we flip it for display and checks).
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

    private Servo leftHolderServo;   // servo for sorting
    private Servo rightHolderServo;  // servo for sorting

    // =========================
    // Camera (Driver Station live view)
    // =========================
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProc; // optional

    private static final String WEBCAM_NAME = "Webcam 1";

    // =========================
    // Shooter Velocity Constants
    // =========================
    // goBILDA 5202/Yellow Jacket motor encoder is commonly 28 ticks/rev at motor shaft.
    // If your RPM telemetry looks way off, confirm your encoder CPR.
    private static final double SHOOTER_TICKS_PER_REV = 28.0;

    // Start target RPM (tune this)
    private static final double SHOOTER_TARGET_RPM = 1950.0;

    // If you want to scale the target (like a multiplier)
    private static final double SHOOTER_SCALE = 1.0;

    // Allow fire only when within tolerance (recommended)
    private static final boolean REQUIRE_AT_SPEED_TO_FIRE = true;
    private static final double AT_SPEED_TOL_RPM = 150.0;
    private static final double SPINUP_TIMEOUT_SEC = 1.5;

    // IMPORTANT: command signs (because your two shooter motors face each other)
    // Left motor: positive velocity = shooting direction
    // Right motor: NEGATIVE velocity = shooting direction (so encoder will be negative raw)
    private static final int LEFT_CMD_SIGN = +1;
    private static final int RIGHT_CMD_SIGN = -1;
    private static final double LEFT_TRIM  = 1.00;  // start 1.00, tune down if left runs high
    private static final double RIGHT_TRIM = 1.00;  // tune up if right runs low


    // =========================
    // Servo positions
    // =========================
    private static final double SERVO_FIRE_POS = 0.25;
    private static final double SERVO_REST_POS = 0.10;

    // Trigger thresholds
    private static final double SPIN_THRESHOLD = 0.08;
    private static final double FIRE_THRESHOLD = 0.92;
    private static final double RESET_THRESHOLD = 0.40;

    // Servo timing (milliseconds)
    private static final long FIRE_PULSE_MS = 220;

    // State machine
    private enum FireState { IDLE, SPINNING, FIRING, RECOVER }
    private FireState state = FireState.IDLE;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime spinupTimer = new ElapsedTime();
    private boolean firedThisPress = false;

    @Override
    public void init() {
        // Drivetrain
        robot.init(hardwareMap);

        // Shooter mapping
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft"); // Expansion Hub DC #0
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight"); // Expansion Hub DC #1
        tipperServo  = hardwareMap.get(Servo.class, "tipperServo"); // Control Hub Servo #0
        intakeMotor  = hardwareMap.get(DcMotorEx.class, "intakeMotor"); // Expansion Hub DC #2

        leftHolderServo  = hardwareMap.get(Servo.class, "leftHolderServo"); // Control Hub Servo # 2
        rightHolderServo = hardwareMap.get(Servo.class, "rightHolderServo"); // Control Hub Servo # 1

        servoTrapLeft  = hardwareMap.get(CRServo.class, "servoTrapLeft"); // Control Hub Servo #5
        servoTrapRight = hardwareMap.get(CRServo.class, "servoTrapRight"); // Expansion Hub Servo #5

        // =========================
        // Shooter motor setup for setVelocity()
        // =========================
        // Make BOTH motors FORWARD. We handle opposite wheel direction by commanding RIGHT negative velocity.
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Encoders needed for RUN_USING_ENCODER velocity control
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
        leftHolderServo.setPosition(0.53);
        rightHolderServo.setPosition(0.00);

        // Stop shooter explicitly
        setShooterRpm(0.0);

        // =========================
        // Camera init for Driver Station live view
        // =========================
        tagProc = AprilTagProcessor.easyCreateWithDefaults(); // optional
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(tagProc)          // optional
                .enableLiveView(true)           // DS Camera Stream
                .build();

        telemetry.addLine("OB TeleOp + Shooter (Velocity) ready");
        telemetry.addLine("Right trigger: slight press spins up, full press fires");
        telemetry.addData("Target RPM", "%.0f", SHOOTER_TARGET_RPM);
        telemetry.addLine("NOTE: Both motors set FORWARD; right commanded negative internally.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // =========================
        // Normal drive
        // =========================
        boolean squaredInputs = true;
        double speedMult = gamepad1.left_bumper ? 0.55 : 0.55;
        robot.driveFromGamepad(gamepad1, squaredInputs, speedMult);

        // =========================
        // Trigger value
        // =========================
        double trig = Range.clip(gamepad1.right_trigger, 0.0, 1.0);

        // =========================
        // Trap CR servos
        // =========================
        if (gamepad1.b) {
            servoTrapLeft.setPower(-1.0);
            servoTrapRight.setPower(0.2);
        } else {
            servoTrapLeft.setPower(0);
            servoTrapRight.setPower(0);
        }

        // =========================
        // Holder servos
        // =========================
        if (gamepad1.dpad_left) {
            leftHolderServo.setPosition(0.22);
        } else if (gamepad1.dpad_down) {
            leftHolderServo.setPosition(0.53);
        }

        if (gamepad1.dpad_right) {
            rightHolderServo.setPosition(0.0);
        } else if (gamepad1.dpad_up) {
            rightHolderServo.setPosition(0.45);
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
        // NOTE: This overrides shooter/servo trap behavior while held.
        // =========================
        if (gamepad1.y) {
            // Low reverse RPM to clear/unstick (tune if needed)
            setShooterRpm(-900.0);

            servoTrapLeft.setPower(.20);
            servoTrapRight.setPower(-0.20);

            // Show telemetry and skip normal state machine while Y held
            telemetry.addData("OVERRIDE", "Y held (reverse clear)");
            telemetry.update();
            return;
        }

        // =========================
        // State machine (same behavior as your original, but setVelocity)
        // =========================
        switch (state) {
            case IDLE:
                setShooterRpm(0.0);
                tipperServo.setPosition(SERVO_REST_POS);
                firedThisPress = false;

                if (trig >= SPIN_THRESHOLD) {
                    setShooterRpm(SHOOTER_TARGET_RPM);
                    spinupTimer.reset();
                    state = FireState.SPINNING;
                }
                break;

            case SPINNING:
                setShooterRpm(SHOOTER_TARGET_RPM);

                boolean atSpeed = shooterAtSpeed(SHOOTER_TARGET_RPM, AT_SPEED_TOL_RPM);
                boolean timedOut = spinupTimer.seconds() >= SPINUP_TIMEOUT_SEC;

                if (trig >= FIRE_THRESHOLD && !firedThisPress) {
                    if (!REQUIRE_AT_SPEED_TO_FIRE || atSpeed || timedOut) {
                        tipperServo.setPosition(SERVO_FIRE_POS);
                        timer.reset();
                        firedThisPress = true;
                        state = FireState.FIRING;
                    }
                } else if (trig < SPIN_THRESHOLD) {
                    state = FireState.IDLE;
                }
                break;

            case FIRING:
                setShooterRpm(SHOOTER_TARGET_RPM);

                if (timer.milliseconds() >= FIRE_PULSE_MS) {
                    tipperServo.setPosition(SERVO_REST_POS);
                    state = FireState.RECOVER;
                }
                break;

            case RECOVER:
                setShooterRpm(SHOOTER_TARGET_RPM);

                if (trig < RESET_THRESHOLD) {
                    if (trig >= SPIN_THRESHOLD) {
                        state = FireState.SPINNING;
                        firedThisPress = false;
                        spinupTimer.reset();
                    } else {
                        state = FireState.IDLE;
                    }
                }
                break;
        }

        // =========================
        // Optional: show camera state and detection count
        // =========================
        telemetry.addData("Camera", visionPortal == null ? "null" : visionPortal.getCameraState());
        telemetry.addData("Tag detections", (tagProc == null || tagProc.getDetections() == null) ? 0 : tagProc.getDetections().size());

        // =========================
        // Shooter telemetry (RAW + corrected)
        // =========================
        telemetry.addData("Shooter L pos", shooterLeft.getCurrentPosition());
        telemetry.addData("Shooter R pos", shooterRight.getCurrentPosition());

        double velLTpsRaw = shooterLeft.getVelocity();   // ticks/sec
        double velRTpsRaw = shooterRight.getVelocity();  // ticks/sec (will likely be negative while shooting)

        telemetry.addData("Shooter L vel t/s (raw)", "%.0f", velLTpsRaw);
        telemetry.addData("Shooter R vel t/s (raw)", "%.0f", velRTpsRaw);

        telemetry.addData("Shooter Target RPM", "%.0f", SHOOTER_TARGET_RPM);
        telemetry.addData("Shooter L RPM (shooting+)", "%.0f", getShooterRpmShootPositive(shooterLeft, LEFT_CMD_SIGN));
        telemetry.addData("Shooter R RPM (shooting+)", "%.0f", getShooterRpmShootPositive(shooterRight, RIGHT_CMD_SIGN));
        telemetry.addData("At speed?", shooterAtSpeed(SHOOTER_TARGET_RPM, AT_SPEED_TOL_RPM));

        // Driver telemetry
        telemetry.addData("Trigger", "%.3f", trig);
        telemetry.addData("State", state);
        telemetry.addData("Servo", tipperServo.getPosition() >= (SERVO_FIRE_POS - 0.02) ? "FIRING" : "REST");
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
    // Shooter helpers (ticks/sec to avoid unit confusion)
    // =========================
    private void setShooterRpm(double rpm) {
        // Allow negative RPM for reverse-clearing
        double scaledRpm = rpm * SHOOTER_SCALE;

        double tps = rpmToTicksPerSec(rpm);

        shooterLeft.setVelocity(LEFT_CMD_SIGN  * tps * LEFT_TRIM);
        shooterRight.setVelocity(RIGHT_CMD_SIGN * tps * RIGHT_TRIM);

    }

    private boolean shooterAtSpeed(double targetRpm, double tolRpm) {
        double l = getShooterRpmShootPositive(shooterLeft, LEFT_CMD_SIGN);
        double r = getShooterRpmShootPositive(shooterRight, RIGHT_CMD_SIGN);

        // Only meaningful when target is positive (shooting)
        double tgt = Math.abs(targetRpm);

        return Math.abs(l - tgt) <= tolRpm && Math.abs(r - tgt) <= tolRpm;
    }

    /**
     * Returns RPM where "shooting direction" is positive, regardless of raw encoder sign.
     * We multiply the raw ticks/sec by cmdSign so the number you see is intuitive and comparable.
     */
    private double getShooterRpmShootPositive(DcMotorEx m, int cmdSign) {
        double tpsRaw = m.getVelocity(); // ticks/sec raw
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
