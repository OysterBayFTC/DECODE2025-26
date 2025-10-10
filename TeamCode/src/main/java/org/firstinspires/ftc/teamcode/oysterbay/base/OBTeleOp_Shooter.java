// TeamCode/src/main/java/org/firstinspires/ftc/teamcode/oysterbay/teleop/OBTeleOp_Shooter.java
package org.firstinspires.ftc.teamcode.oysterbay.base;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.oysterbay.base.RobotStructure;

@TeleOp(name = "OB TeleOp + Shooter", group = "OB")
public class OBTeleOp_Shooter extends OpMode {
    private final RobotStructure robot = new RobotStructure();

    // Shooter motors (configure these device names in your RC configuration)
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    // Optional: scale overall shooter power if needed
    private static final double SHOOTER_SCALE = 1.0; // 1.0 = full, 0.8 = cap at 80%

    @Override
    public void init() {
        // Drive train
        robot.init(hardwareMap);

        // Shooter mapping
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        // Make positive power spin the wheels INWARD to fire the ball between them:
        // Set one side reversed (adjust if your physical spin is opposite)
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Shooters usually run without encoders and coast when power = 0
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("OB TeleOp + Shooter initialized");
        telemetry.addLine("LT = reverse, RT = shoot (variable speed), LB = slow drive");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Drive: left bumper for slow mode ---
        boolean squaredInputs = true;
        double speedMult = gamepad1.left_bumper ? 0.4 : 1.0;
        robot.driveFromGamepad(gamepad1, squaredInputs, speedMult);

        // --- Shooter control (variable) ---
        // Right trigger => forward (shoot), Left trigger => reverse
        double shootFwd = gamepad1.right_trigger; // 0..1
        double shootRev = gamepad1.left_trigger;  // 0..1
        double shooterPower = Range.clip((shootFwd - shootRev) * SHOOTER_SCALE, -1.0, 1.0);

        shooterLeft.setPower(shooterPower);
        shooterRight.setPower(shooterPower); // opposite spin due to RIGHT motor reversed

        // Telemetry
        telemetry.addData("Drive slow", speedMult < 1.0);
        telemetry.addData("Shooter", "%.2f", shooterPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Ensure shooter stops
        if (shooterLeft != null)  shooterLeft.setPower(0);
        if (shooterRight != null) shooterRight.setPower(0);
        robot.setDriverPowerZERO();
    }
}
