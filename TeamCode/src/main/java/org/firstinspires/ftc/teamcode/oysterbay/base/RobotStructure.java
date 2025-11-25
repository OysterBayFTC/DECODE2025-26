package org.firstinspires.ftc.teamcode.oysterbay.base;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp-only drivetrain structure to be reused across programs.
 * Owns the four drive motors and exposes simple helpers.
 *
 * Motor names expected in RC config:
 *  - "motorFrontRight", "motorFrontLeft", "motorBackRight", "motorBackLeft"
 */
public class RobotStructure {

    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;

    // --- Tunables ---
    private static final double DEADBAND = 0.05;  // stick deadzone
    private static final double EXPO_DRIVE = 2.0; // >1 softens low-end; 1.0 = linear

    public void init(HardwareMap hardwareMap) {
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorFrontLeft  = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackRight  = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        motorBackLeft   = hardwareMap.get(DcMotorEx.class, "motorBackLeft");

        // Make +power = forward for all wheels. Adjust if needed for your build.
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Drive from a gamepad (left stick = translation, right stick X = turn).
     *
     * @param gp the driver gamepad
     * @param squaredInputs true to apply exponential shaping (smoother low speed)
     * @param speedMult overall speed scale (e.g., 1.0 normal, 0.4 slow)
     */
    public void driveFromGamepad(Gamepad gp, boolean squaredInputs, double speedMult) {
        // FTC convention: +y forward, +x right, +r CCW
        double y = -gp.left_stick_y;   // invert for typical forward on push
        double x =  gp.left_stick_x;
        double r =  gp.right_stick_x;

        // Deadband
        y = applyDeadband(y, DEADBAND);
        x = applyDeadband(x, DEADBAND);
        r = applyDeadband(r, DEADBAND);

        // Expo shaping (optional)
        if (squaredInputs) {
            y = expo(y, EXPO_DRIVE);
            x = expo(x, EXPO_DRIVE);
            r = expo(r, EXPO_DRIVE);
        }

        // Apply speed scale
        y *= speedMult;
        x *= speedMult;
        r *= speedMult;

        // Mecanum mixer
        driveCartesian(x, y, r);
    }

    /** Direct power set by wheel (already normalized/clipped by caller). */
    public void setDriverMotorPower(double frontRight, double frontLeft,
                                    double backRight, double backLeft) {
        motorFrontRight.setPower(Range.clip(frontRight, -1, 1));
        motorFrontLeft.setPower(Range.clip(frontLeft,  -1, 1));
        motorBackRight.setPower(Range.clip(backRight,  -1, 1));
        motorBackLeft.setPower(Range.clip(backLeft,    -1, 1));
    }

    /** Stop all drive motors. */
    public void setDriverPowerZERO() {
        setDriverMotorPower(0, 0, 0, 0);
    }

    /** Simple strafes (normalized). */
    public void translateRight(double m) { driveCartesian( m, 0, 0); }
    public void translateLeft (double m) { driveCartesian(-m, 0, 0); }

    // ---------- internals ----------

    /** Standard mecanum cartesian mixer with normalization. */
    private void driveCartesian(double x, double y, double r) {
        double fl = y + x + r;
        double fr = y - x - r;
        double bl = y - x + r;
        double br = y + x - r;

        // Normalize so the max magnitude is 1.0
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        // Order parameters as (frontRight, frontLeft, backRight, backLeft)
        setDriverMotorPower(fr / max, fl / max, br / max, bl / max);
    }

    private static double applyDeadband(double v, double d) {
        return (Math.abs(v) < d) ? 0.0 : v;
    }

    /** expo = sign(x)*|x|^p; p>=1.0 */
    private static double expo(double v, double p) {
        double s = Math.signum(v);
        return s * Math.pow(Math.abs(v), p);
    }
}
