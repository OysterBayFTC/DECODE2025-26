// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/OBShooterRPMTuner_Velocity.java
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

/**
 * Shooter RPM Fine Tuner (setVelocity) - MATCHES OBTeleOp_Shooter_Velocity systems/logic
 *
 * What this matches from your driver TeleOp:
 *  - Motor names: shooterMotor, shooterMotor2, intakeMotor, upperIntakeMotor, trapServo
 *  - Motor directions:
 *      shooterMotor  FORWARD
 *      shooterMotor2 REVERSE
 *      upperIntakeMotor REVERSE
 *  - setShooterRpm(): sends SAME +tps to both motors (no extra sign flips)
 *  - RPM conversion uses TICKS_PER_AXLE_REV = 28 * gearRatio (same structure)
 *  - Trap servo on gamepad2: a=close(0.05), b=open(0.80) (same as your TeleOp code)
 *
 * Tuner controls (gamepad1):
 *  - Drive: robot.driveFromGamepad(gamepad1, true, 0.55)
 *  - RPM adjust:
 *      RB: +150 RPM
 *      LB: -150 RPM
 *      DpadUp: +10 RPM
 *      DpadDown: -10 RPM
 *      START: reset to DEFAULT_RPM
 *  - Shooter:
 *      RT >= SPIN_THRESHOLD: spin at target RPM
 *      RT >= FIRE_THRESHOLD: pulse tipper (this OpMode uses trapServo as the "tipper/fire" servo)
 *
 * Other:
 *  - Intake (hold): gamepad1.a = intakeMotor -1, upperIntakeMotor 0  (same "A intake only" feel)
 */
@TeleOp(name = "OB Shooter RPM Tuner (Velocity)", group = "OB")
public class OBTeleOp_ShooterRPMTuner extends OpMode {

    private final RobotStructure robot = new RobotStructure();

    // Hardware (MATCHES your TeleOp names)
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;
    private DcMotorEx intakeMotor;
    private DcMotorEx upperIntakeMotor;
    private Servo trapServo;

    // =========================
    // Shooter targets / conversion (MATCHES TeleOp structure)
    // =========================
    private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0;
    private static final double GEAR_RATIO_MOTOR_TO_AXLE = 1.0; // change if needed
    private static final double TICKS_PER_AXLE_REV = ENCODER_TICKS_PER_MOTOR_REV * GEAR_RATIO_MOTOR_TO_AXLE;

    // RPM tuning
    private static final double DEFAULT_RPM = 3000.0;
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 6000.0;

    private static final double COARSE_STEP_RPM = 150.0;
    private static final double FINE_STEP_RPM   = 10.0;

    // Trigger thresholds (same feel concept as driver: light press = spin, full = fire)
    private static final double SPIN_THRESHOLD  = 0.08;
    private static final double FIRE_THRESHOLD  = 0.92;
    private static final double RESET_THRESHOLD = 0.40;

    // Fire timing
    private static final long FIRE_PULSE_MS = 220;

    // Fire positions (you can tune these)
    // Note: Your driver TeleOp uses trapServo positions 0.05 and 0.80 for close/open.
    // For firing, you may want a quick "push" position; defaulting to 0.80 then returning to 0.05.
    private static final double SERVO_FIRE_POS = 0.80; // "Open"
    private static final double SERVO_REST_POS = 0.05; // "Close"

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

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        upperIntakeMotor = hardwareMap.get(DcMotorEx.class, "upperIntakeMotor");
        trapServo = hardwareMap.get(Servo.class, "trapServo");

        // MATCH TeleOp motor directions
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        upperIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // MATCH TeleOp modes
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Keep it consistent with your driver TeleOp: BRAKE on shooter at zero
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0.0);

        upperIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperIntakeMotor.setPower(0.0);

        // Set trap to rest
        trapServo.setPosition(SERVO_REST_POS);

        // Make sure shooter is stopped at init
        setShooterRpm(0.0);

        telemetry.addLine("OB Shooter RPM Tuner (Velocity) ready");
        telemetry.addLine("RB/LB: +/-150 RPM | DpadUp/Down: +/-10 RPM | START: reset");
        telemetry.addLine("RT: spin | Full RT: fire (servo pulse)");
        telemetry.addLine("X: toggle require-at-speed | Dpad L/R: tol +/-10");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Drive (same call style as your TeleOp; speedMult fixed like your original tuner)
        robot.driveFromGamepad(gamepad1, true, 0.55);

        // Intake (match your TeleOp "A intake only" behavior)
        if (gamepad1.a) {
            intakeMotor.setPower(-1.0);
            upperIntakeMotor.setPower(0.0);
        } else {
            intakeMotor.setPower(0.0);
            upperIntakeMotor.setPower(0.0);
        }

        // Trap servo manual (MATCH driver TeleOp mapping)
        if (gamepad2.a) trapServo.setPosition(0.05); // Close
        if (gamepad2.b) trapServo.setPosition(0.80); // Open

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
                        trapServo.setPosition(SERVO_FIRE_POS);
                        timer.reset();
                        firedThisPress = true;
                        state = FireState.FIRING;
                    }
                } else if (trig < SPIN_THRESHOLD) {
                    // release = stop
                    state = FireState.IDLE;
                }
                break;

            case FIRING:
                setShooterRpm(targetRpm);

                if (timer.milliseconds() >= FIRE_PULSE_MS) {
                    trapServo.setPosition(SERVO_REST_POS);
                    state = FireState.RECOVER;
                }
                break;

            case RECOVER:
                setShooterRpm(targetRpm);

                // must release partway before allowing another shot
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

        // Telemetry (MATCH driver TeleOp conversion)
        double m1Rpm = ticksPerSecToRpm(shooterMotor.getVelocity());
        double m2Rpm = ticksPerSecToRpm(shooterMotor2.getVelocity());
        boolean atSpeedNow = shooterAtSpeed(targetRpm, atSpeedTolRpm);

        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("Motor1 RPM", "%.0f", m1Rpm);
        telemetry.addData("Motor2 RPM", "%.0f", m2Rpm);
        telemetry.addData("Tol RPM", "%.0f", atSpeedTolRpm);
        telemetry.addData("At speed?", atSpeedNow);
        telemetry.addData("Require at speed?", requireAtSpeedToFire);
        telemetry.addData("Trigger", "%.2f", trig);
        telemetry.addData("State", state);
        telemetry.addData("TrapServo pos", "%.2f", trapServo.getPosition());

        telemetry.update();
    }

    @Override
    public void stop() {
        setShooterRpm(0.0);

        if (intakeMotor != null) intakeMotor.setPower(0.0);
        if (upperIntakeMotor != null) upperIntakeMotor.setPower(0.0);

        if (trapServo != null) trapServo.setPosition(SERVO_REST_POS);

        robot.setDriverPowerZERO();
    }

    // =========================
    // Shooter helpers (MATCH driver TeleOp sign logic)
    // =========================
    private void setShooterRpm(double rpm) {
        double tps = rpmToTicksPerSec(rpm);
        shooterMotor.setVelocity(tps);
        shooterMotor2.setVelocity(tps);
    }

    private boolean shooterAtSpeed(double targetRpm, double tolRpm) {
        double tgt = Math.abs(targetRpm);
        double l = Math.abs(ticksPerSecToRpm(shooterMotor.getVelocity()));
        double r = Math.abs(ticksPerSecToRpm(shooterMotor2.getVelocity()));
        return Math.abs(l - tgt) <= tolRpm && Math.abs(r - tgt) <= tolRpm;
    }

    private static double rpmToTicksPerSec(double rpm) {
        return (rpm * TICKS_PER_AXLE_REV) / 60.0;
    }

    private static double ticksPerSecToRpm(double tps) {
        return (tps * 60.0) / TICKS_PER_AXLE_REV;
    }
}
