// TeamCode/src/main/java/org/firstinspires/ftc/teamcode/oysterbay/teleop/OBWheelDirectionDebug_DirectMap.java
package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Debug opmode: run each wheel forward independently using A/B/X/Y,
 * and run the same wheel backward using
 * Z0.the D-pad.
 *
 * Mapping (Gamepad1):
 *  Forward:
 *   - A -> Front Left
 *   - B -> Front Right
 *   - X -> Back Left
 *   - Y -> Back Right
 *
 *  Reverse:
 *   - Dpad Left  -> Front Left
 *   - Dpad Right -> Front Right
 *   - Dpad Down  -> Back Left
 *   - Dpad Up    -> Back Right
 *
 * Motor names expected in RC config:
 *  - "motorFrontRight", "motorFrontLeft", "motorBackRight", "motorBackLeft"
 */
@TeleOp(name = "OB Debug: Wheel Dir (Direct Map)", group = "OB")
public class OBWheelDirectionDebug extends OpMode {

    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;

    private static final double TEST_POWER = 0.35;

    @Override
    public void init() {
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorFrontLeft  = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackRight  = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        motorBackLeft   = hardwareMap.get(DcMotorEx.class, "motorBackLeft");

        // Match your current RobotStructure behavior:
        // Make +power = forward for all wheels (right side reversed on many builds).
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Optional (clean test behavior)
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Wheel Direction Debug Ready (Direct Map)");
        telemetry.addLine("Hold A/B/X/Y to run one wheel forward.");
        telemetry.addLine("Hold Dpad to run the same wheel backward.");
        telemetry.addLine("A=FL, B=FR, X=BL, Y=BR");
        telemetry.addLine("DpadLeft=FL-, DpadRight=FR-, DpadDown=BL-, DpadUp=BR-");
        telemetry.update();
    }

    @Override
    public void loop() {
        double fl = 0, fr = 0, bl = 0, br = 0;

        // Forward (ABXY)
        if (gamepad1.a) fl = +TEST_POWER; // Front Left
        if (gamepad1.b) fr = +TEST_POWER; // Front Right
        if (gamepad1.x) bl = +TEST_POWER; // Back Left
        if (gamepad1.y) br = +TEST_POWER; // Back Right

        // Reverse (D-pad) overrides if both held
        if (gamepad1.dpad_left)  fl = -TEST_POWER; // Front Left
        if (gamepad1.dpad_right) fr = -TEST_POWER; // Front Right
        if (gamepad1.dpad_down)  bl = -TEST_POWER; // Back Left
        if (gamepad1.dpad_up)    br = -TEST_POWER; // Back Right

        fl = Range.clip(fl, -1, 1);
        fr = Range.clip(fr, -1, 1);
        bl = Range.clip(bl, -1, 1);
        br = Range.clip(br, -1, 1);

        motorFrontLeft.setPower(fl);
        motorFrontRight.setPower(fr);
        motorBackLeft.setPower(bl);
        motorBackRight.setPower(br);

        telemetry.addLine("Hold one button at a time for clean results.");
        telemetry.addData("FL (A / DpadLeft)",  fl);
        telemetry.addData("FR (B / DpadRight)", fr);
        telemetry.addData("BL (X / DpadDown)",  bl);
        telemetry.addData("BR (Y / DpadUp)",    br);
        telemetry.update();
    }

    @Override
    public void stop() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
}
