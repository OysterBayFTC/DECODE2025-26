// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/test/PinpointPrintOnly.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Pinpoint Print ONLY", group = "Test")
public class PinpointPrintOnly extends LinearOpMode {

    // MUST match your Robot Config name
    private static final String PINPOINT_NAME = "pinpoint";

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        // If you already set offsets/directions in another OpMode, you can still set them here.
        // If you DON'T want to change anything, you can delete these 3 lines.
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // pinpoint.setOffsets(-157.5, 15.0);
        // pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
        //                              GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Optional: zero at start (comment out if you don't want it resetting)
        // pinpoint.resetPosAndIMU();

        telemetry.addLine("Pinpoint Print ONLY ready.");
        telemetry.addLine("Press START, then move robot by hand.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            telemetry.addData("Status", String.valueOf(pinpoint.getDeviceStatus()));

            if (pose != null) {
                telemetry.addData("X (mm)", "%.1f", pose.getX(DistanceUnit.MM));
                telemetry.addData("Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
                telemetry.addData("Heading (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));

                // Optional inches
                telemetry.addData("X (in)", "%.2f", pose.getX(DistanceUnit.MM) / 25.4);
                telemetry.addData("Y (in)", "%.2f", pose.getY(DistanceUnit.MM) / 25.4);
            } else {
                telemetry.addLine("Pose is null (Pinpoint not returning pose yet).");
            }

            telemetry.update();
            idle();
        }
    }
}
