package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@TeleOp(name = "Odometry Readout (Pinpoint/RR)", group = "Debug")
public class PinpointOdoReadout extends LinearOpMode {

    // Road Runner pose units: inches + radians
    private static final Pose2d INITIAL_POSE = new Pose2d(0.0, 0.0, 0.0);

    @Override
    public void runOpMode() {
        // This should match your existing working RoadRunner + Pinpoint setup
        PinpointDrive drive = new PinpointDrive(hardwareMap, INITIAL_POSE);

        // INIT loop: lets you physically move the robot and see live pose before START
        while (!isStarted() && !isStopRequested()) {
            drive.updatePoseEstimate();

            Pose2d p = drive.pose;
            telemetry.addData("X (in)", "%.3f", p.position.x);
            telemetry.addData("Y (in)", "%.3f", p.position.y);
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(p.heading.toDouble()));
            telemetry.update();
            idle();
        }

        if (isStopRequested()) return;

        // RUN loop: keep updating + printing
        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            Pose2d p = drive.pose;
            telemetry.addData("X (in)", "%.3f", p.position.x);
            telemetry.addData("Y (in)", "%.3f", p.position.y);
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(p.heading.toDouble()));
            telemetry.update();
            idle();
        }
    }
}
