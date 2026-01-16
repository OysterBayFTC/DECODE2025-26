// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/OBShooterRPMTuner_Velocity.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.oysterbay.base.RobotStructure;

/**
 * Shooter RPM Fine Tuner (setVelocity version that matches your current shooter sign logic)
 *
 * Goal: park in one place, repeatedly shoot, and tune RPM until arc is perfect.
 *
 * Controls (gamepad1):
 *  - Drive: normal RobotStructure drive
 *  - Intake: A (hold)
 *  - Trap:   B (hold)  (same as your TeleOp)
 *
 * Shooter:
 *  - Right trigger: slight press spins up, full press fires (same behavior as driver)
 *
 * RPM adjust (4 buttons total):
 *  - Right bumper: +150 RPM
 *  - Left bumper:  -150 RPM
 *  - Dpad up:      +10 RPM
 *  - Dpad down:    -10 RPM
 *
 * Optional tuning helpers:
 *  - X: toggle REQUIRE_AT_SPEED_TO_FIRE (lets you choose “always fire” vs “fire only at speed”)
 *  - Dpad left/right: adjust AT_SPEED_TOL_RPM by +/-10 (how tight “at speed” is)
 *  - START: reset RPM to default
 *
 * IMPORTANT: This OpMode assumes your fixed wiring outcome:
 *  - Both shooter motors set Direction.FORWARD
 *  - Right motor encoder/velocity is negative in the shooting direction
 *  - We command RIGHT motor with negative setVelocity and "flip" it for telemetry checks
 */
@TeleOp(name = "OB Shooter RPM Tuner (Velocity)", group = "OB")
public class OBTeleOp_ShooterRPMTuner extends OpMode {

    private final RobotStructure robot = new RobotStructure();

    // Hardware
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private DcMotorEx intakeMotor;
    private Servo tipperServo;
    private CRServo servoTrapLeft;
    private CRServo servoTrapRight;

    // =========================
    // Shooter config (edit these as needed)
    // =========================
    private static final String SHOOTER_LEFT_NAME  = "shooterLeft";
    private static final String SHOOTER_RIGHT_NAME = "shooterRight";
    private static final String INTAKE_NAME        = "intakeMotor";
    private static final String TIPPER_NAME        = "tipperServo";
    private static final String TRAP_LEFT_NAME     = "servoTrapLeft";
    private static final String TRAP_RIGHT_NAME    = "servoTrapRight";

    // Encoder ticks/rev (verify for your exact motor)
    private static final double SHOOTER_TICKS_PER_REV = 28.0;

    // Sign convention (matches the “current logic” we settled on)
    private static final int LEFT_CMD_SIGN  = +1;
    private static final int RIGHT_CMD_SIGN = -1;

    // RPM tuning
    private static final double DEFAULT_RPM = 4500.0;
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 6000.0;

    private static final double COARSE_STEP_RPM = 150.0;
    private static final double FINE_STEP_RPM   = 10.0;

    // Servo positions
    private static final double SERVO_FIRE_POS = 0.25;
    private static final double SERVO_REST_POS = 0.10;

    // Trigger thresholds (same feel as driver)
    private static final double SPIN_THRESHOLD  = 0.08;
    private static final double FIRE_THRESHOLD  = 0.92;
    private static final double RESET_THRESHOLD = 0.40;

    // Fire timing
    private static final long FIRE_PULSE_MS = 220;

    // Optional: firing gate
    private boolean requireAtSpeedToFire = true;
    private double atSpeedTolRpm = 150.0;
    private static final double SPINUP_TIMEOUT_SEC = 1.5;

    // State machine
    private enum FireState { IDLE, SPINNING, FIRING, RECOVER }
    private FireState state = FireState.IDLE;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime spinupTimer = new ElapsedTime();
    private boolean firedThisPress = false;

    // Tunables
    private double targetRpm = DEFAULT_RPM;

    // Edge detection
    private boolean prevRB = false, prevLB = false, prevDU = false, prevDD = false;
    private boolean prevDL = false, prevDR = false;
    private boolean prevX = false;
    private boolean prevStart = false;

    @Override
    public void init() {
        robot.init(hardwareMap);

        shooterLeft  = hardwareMap.get(DcMotorEx.class, SHOOTER_LEFT_NAME);
        shooterRight = hardwareMap.get(DcMotorEx.class, SHOOTER_RIGHT_NAME);
        intakeMotor  = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
        tipperServo  = hardwareMap.get(Servo.class, TIPPER_NAME);

        servoTrapLeft  = hardwareMap.get(CRServo.class, TRAP_LEFT_NAME);
        servoTrapRight = hardwareMap.get(CRServo.class, TRAP_RIGHT_NAME);

        // Shooter motor setup for velocity control
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Intake setup
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tipperServo.setPosition(SERVO_REST_POS);

        setShooterRpm(0.0);

        telemetry.addLine("OB Shooter RPM Tuner (Velocity) ready");
        telemetry.addLine("RB/LB: +/-150 RPM | DpadUp/Down: +/-10 RPM");
        telemetry.addLine("RT: spin | Full RT: fire (same feel as driver)");
        telemetry.addLine("X: toggle require-at-speed | Dpad L/R: tol +/-10 | START: reset RPM");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Drive (keep your normal feel)
        boolean squaredInputs = true;
        double speedMult = gamepad1.left_bumper ? 0.55 : 0.55;
        robot.driveFromGamepad(gamepad1, squaredInputs, speedMult);

        // Intake (hold)
        if (gamepad1.a) intakeMotor.setPower(-1.0);
        else intakeMotor.setPower(0.0);

        // Trap (hold)
        if (gamepad1.b) {
            servoTrapLeft.setPower(-1.0);
            servoTrapRight.setPower(0.2);
        } else {
            servoTrapLeft.setPower(0);
            servoTrapRight.setPower(0);
        }

        // === RPM adjust buttons ===
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;
        boolean du = gamepad1.dpad_up;
        boolean dd = gamepad1.dpad_down;
        boolean dl = gamepad1.dpad_left;
        boolean dr = gamepad1.dpad_right;

        if (rb && !prevRB) targetRpm += COARSE_STEP_RPM;
        if (lb && !prevLB) targetRpm -= COARSE_STEP_RPM;
        if (du && !prevDU) targetRpm += FINE_STEP_RPM;
        if (dd && !prevDD) targetRpm -= FINE_STEP_RPM;

        // Optional: adjust tolerance with left/right
        if (dr && !prevDR) atSpeedTolRpm += 10.0;
        if (dl && !prevDL) atSpeedTolRpm -= 10.0;

        prevRB = rb; prevLB = lb; prevDU = du; prevDD = dd; prevDL = dl; prevDR = dr;

        targetRpm = Range.clip(targetRpm, MIN_RPM, MAX_RPM);
        atSpeedTolRpm = Range.clip(atSpeedTolRpm, 20.0, 400.0);

        // Toggle require-at-speed
        boolean x = gamepad1.x;
        if (x && !prevX) requireAtSpeedToFire = !requireAtSpeedToFire;
        prevX = x;

        // Reset RPM
        boolean start = gamepad1.start;
        if (start && !prevStart) targetRpm = DEFAULT_RPM;
        prevStart = start;

        // Trigger
        double trig = Range.clip(gamepad1.right_trigger, 0.0, 1.0);

        // State machine (velocity-based)
        switch (state) {
            case IDLE:
                setShooterRpm(0.0);
                tipperServo.setPosition(SERVO_REST_POS);
                firedThisPress = false;

                if (trig >= SPIN_THRESHOLD) {
                    setShooterRpm(targetRpm);
                    spinupTimer.reset();
                    state = FireState.SPINNING;
                }
                break;

            case SPINNING:
                setShooterRpm(targetRpm);

                boolean atSpeed = shooterAtSpeed(targetRpm, atSpeedTolRpm);
                boolean timedOut = spinupTimer.seconds() >= SPINUP_TIMEOUT_SEC;

                if (trig >= FIRE_THRESHOLD && !firedThisPress) {
                    if (!requireAtSpeedToFire || atSpeed || timedOut) {
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
                setShooterRpm(targetRpm);

                if (timer.milliseconds() >= FIRE_PULSE_MS) {
                    tipperServo.setPosition(SERVO_REST_POS);
                    state = FireState.RECOVER;
                }
                break;

            case RECOVER:
                setShooterRpm(targetRpm);

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

        // Telemetry (show corrected “shooting+” RPM for both sides)
        double lRpm = getShooterRpmShootPositive(shooterLeft, LEFT_CMD_SIGN);
        double rRpm = getShooterRpmShootPositive(shooterRight, RIGHT_CMD_SIGN);
        boolean atSpeedNow = shooterAtSpeed(targetRpm, atSpeedTolRpm);

        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("L RPM (shooting+)", "%.0f", lRpm);
        telemetry.addData("R RPM (shooting+)", "%.0f", rRpm);
        telemetry.addData("Tol RPM", "%.0f", atSpeedTolRpm);
        telemetry.addData("At speed?", atSpeedNow);
        telemetry.addData("Require at speed?", requireAtSpeedToFire);
        telemetry.addData("Trigger", "%.2f", trig);
        telemetry.addData("State", state);
        telemetry.addData("Tipper", tipperServo.getPosition() >= (SERVO_FIRE_POS - 0.02) ? "FIRING" : "REST");

        // Raw encoder signs for debugging
        telemetry.addData("Raw L pos", shooterLeft.getCurrentPosition());
        telemetry.addData("Raw R pos", shooterRight.getCurrentPosition());
        telemetry.addData("Raw L vel (t/s)", "%.0f", shooterLeft.getVelocity());
        telemetry.addData("Raw R vel (t/s)", "%.0f", shooterRight.getVelocity());

        telemetry.update();
    }

    @Override
    public void stop() {
        setShooterRpm(0.0);
        if (intakeMotor != null) intakeMotor.setPower(0.0);
        if (servoTrapLeft != null) servoTrapLeft.setPower(0.0);
        if (servoTrapRight != null) servoTrapRight.setPower(0.0);
        if (tipperServo != null) tipperServo.setPosition(SERVO_REST_POS);
        robot.setDriverPowerZERO();
    }

    // =========================
    // Shooter helpers (ticks/sec)
    // =========================
    private void setShooterRpm(double rpm) {
        // Allow reverse if you ever want it; for tuning you’ll keep rpm positive.
        double tps = rpmToTicksPerSec(rpm);

        // Apply explicit physical sign convention
        shooterLeft.setVelocity(LEFT_CMD_SIGN * tps);
        shooterRight.setVelocity(RIGHT_CMD_SIGN * tps);
    }

    private boolean shooterAtSpeed(double targetRpm, double tolRpm) {
        double tgt = Math.abs(targetRpm);
        double l = getShooterRpmShootPositive(shooterLeft, LEFT_CMD_SIGN);
        double r = getShooterRpmShootPositive(shooterRight, RIGHT_CMD_SIGN);
        return Math.abs(l - tgt) <= tolRpm && Math.abs(r - tgt) <= tolRpm;
    }

    /**
     * Returns RPM where "shooting direction" is positive, regardless of raw encoder sign.
     */
    private double getShooterRpmShootPositive(DcMotorEx m, int cmdSign) {
        double tpsRaw = m.getVelocity();        // ticks/sec raw (right likely negative)
        double tpsShootPos = tpsRaw * cmdSign;  // flip so shooting is positive
        return ticksPerSecToRpm(tpsShootPos);
    }

    private static double rpmToTicksPerSec(double rpm) {
        return (rpm * SHOOTER_TICKS_PER_REV) / 60.0;
    }

    private static double ticksPerSecToRpm(double tps) {
        return (tps * 60.0) / SHOOTER_TICKS_PER_REV;
    }
}
