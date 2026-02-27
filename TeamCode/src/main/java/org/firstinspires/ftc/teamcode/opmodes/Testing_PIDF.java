// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/tuning/ShooterVelocityPIDFTuner.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Shooter Velocity PIDF Tuner (DcMotorEx)
 *
 * - Tunes ONLY P and F for RUN_USING_ENCODER velocity control.
 * - I and D are constants captured at init (not tunable, not exposed).
 * - Starts P/F from the motor's reported PIDF (often 0,0,0,0 on FTC hubs).
 *
 * Controls (gamepad1):
 * - A: Toggle motor ON/OFF
 * - Y: Emergency stop (motor OFF + target velocity = 0)
 * - X: Zero target velocity
 *
 * - Dpad Up/Down: Toggle selected coefficient (P <-> F)
 * - Dpad Left/Right: Change step size
 * - LB: Decrease selected coefficient by step
 * - RB: Increase selected coefficient by step
 *
 * - Left stick Y: Adjust target velocity (ticks/sec)
 *
 * Extra:
 * - B: runs trap servos (copied from your code)
 */
@Config
@TeleOp(name = "TUNE Shooter Velocity PIDF (P/F only)", group = "Tuning")
public class Testing_PIDF extends OpMode {

    // =========================
    // Dashboard-editable values
    // =========================
    public static double kP = 0.0;
    public static double kF = 0.0;

    // Target velocity in encoder ticks/sec (NOT RPM).
    public static double targetVelTicksPerSec = 1500.0;

    // Limits
    public static double maxVelTicksPerSec = 7000.0;

    // If your motor spins the wrong direction, set -1
    public static int directionSign = 1;

    // =========================
    // Internal state
    // =========================
    private DcMotorEx shooterMotor;
    private CRServo servoTrapLeft;
    private CRServo servoTrapRight;

    private boolean motorEnabled = false;

    private enum Coeff { P, F }
    private Coeff selected = Coeff.P;

    private final double[] stepOptions = new double[] {0.0005, 0.005, 0.05, 0.5, 5.0};
    private int stepIndex = 1;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime buttonTimer = new ElapsedTime();

    // Edge detection
    private boolean prevA, prevX, prevY, prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight, prevLB, prevRB;

    // Last-applied cache
    private double lastP = Double.NaN, lastI = Double.NaN, lastD = Double.NaN, lastF = Double.NaN;

    // I and D constants captured at init
    private double constI = 0.0;
    private double constD = 0.0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoTrapLeft = hardwareMap.get(CRServo.class, "servoTrapLeft");
        servoTrapRight = hardwareMap.get(CRServo.class, "servoTrapRight");

        // Capture current PIDF from the controller (often returns zeros on FTC hubs).
        PIDFCoefficients current = shooterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        if (current != null) {
            kP = current.p;
            kF = current.f;
            constI = current.i;
            constD = current.d;
        }

        // Apply once so caches initialize + motor has what we think it has
        applyVelocityPIDF(true);

        telemetry.addLine("Shooter Velocity PIDF Tuner ready (P/F only).");
        telemetry.addLine("A toggle | stick sets target | dpad up/down select P<->F | dpad left/right step | LB/RB adjust | X zero | Y STOP");
        telemetry.update();

        loopTimer.reset();
        buttonTimer.reset();
    }

    @Override
    public void loop() {
        handleButtons();
        handleTargetAdjust();

        applyVelocityPIDF(false);
        runShooter();

        sendTelemetry();

        loopTimer.reset();
    }

    private void handleButtons() {
        boolean allow = buttonTimer.milliseconds() > 120;

        boolean a = gamepad1.a;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        if (allow && a && !prevA) {
            motorEnabled = !motorEnabled;
            buttonTimer.reset();
        }

        if (allow && y && !prevY) {
            motorEnabled = false;
            targetVelTicksPerSec = 0.0;
            buttonTimer.reset();
        }

        if (allow && x && !prevX) {
            targetVelTicksPerSec = 0.0;
            buttonTimer.reset();
        }

        // Toggle selected coefficient (P <-> F)
        if (allow && dpadUp && !prevDpadUp) {
            selected = (selected == Coeff.P) ? Coeff.F : Coeff.P;
            buttonTimer.reset();
        }
        if (allow && dpadDown && !prevDpadDown) {
            selected = (selected == Coeff.P) ? Coeff.F : Coeff.P;
            buttonTimer.reset();
        }

        // Step size
        if (allow && dpadRight && !prevDpadRight) {
            stepIndex = Math.min(stepIndex + 1, stepOptions.length - 1);
            buttonTimer.reset();
        }
        if (allow && dpadLeft && !prevDpadLeft) {
            stepIndex = Math.max(stepIndex - 1, 0);
            buttonTimer.reset();
        }

        double step = stepOptions[stepIndex];
        if (allow && rb && !prevRB) {
            bumpSelected(+step);
            buttonTimer.reset();
        }
        if (allow && lb && !prevLB) {
            bumpSelected(-step);
            buttonTimer.reset();
        }

        // Your trap servo control from the original code
        if (gamepad1.b) {
            servoTrapLeft.setPower(-0.6);
            servoTrapRight.setPower(0.6);
        } else {
            servoTrapLeft.setPower(0);
            servoTrapRight.setPower(0);
        }

        prevA = a; prevX = x; prevY = y;
        prevDpadUp = dpadUp; prevDpadDown = dpadDown; prevDpadLeft = dpadLeft; prevDpadRight = dpadRight;
        prevLB = lb; prevRB = rb;
    }

    private void handleTargetAdjust() {
        double dt = Math.max(loopTimer.seconds(), 0.02);
        double stick = -gamepad1.left_stick_y; // up = positive
        double delta = stick * 2500.0 * dt;
        targetVelTicksPerSec = Range.clip(targetVelTicksPerSec + delta, -maxVelTicksPerSec, maxVelTicksPerSec);
    }

    private void runShooter() {
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (!motorEnabled) {
            shooterMotor.setPower(0.0);
            return;
        }

        shooterMotor.setVelocity(directionSign * targetVelTicksPerSec);
    }

    private void applyVelocityPIDF(boolean force) {
        double p = kP;
        double i = constI;   // constant
        double d = constD;   // constant
        double f = kF;

        boolean changed = force
                || p != lastP
                || i != lastI
                || d != lastD
                || f != lastF;

        if (!changed) return;

        PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        lastP = p; lastI = i; lastD = d; lastF = f;
    }

    private void bumpSelected(double delta) {
        switch (selected) {
            case P:
                kP = Math.max(0.0, kP + delta);
                break;
            case F:
                kF = Math.max(0.0, kF + delta);
                break;
        }
    }

    private void sendTelemetry() {
        double actualVel = shooterMotor.getVelocity();

        telemetry.addData("Motor", "shooterMotor");
        telemetry.addData("Enabled", motorEnabled);

        telemetry.addData("Selected", selected);
        telemetry.addData("Step", stepOptions[stepIndex]);

        telemetry.addData("kP", kP);
        telemetry.addData("kF", kF);

        telemetry.addData("I const", constI);
        telemetry.addData("D const", constD);

        telemetry.addData("Target (ticks/s)", directionSign * targetVelTicksPerSec);
        telemetry.addData("Actual (ticks/s)", actualVel);

        telemetry.addLine("A toggle | stick sets target | dpad up/down select P<->F | dpad left/right step | LB/RB adjust | X zero | Y STOP");
        telemetry.update();
    }
}
