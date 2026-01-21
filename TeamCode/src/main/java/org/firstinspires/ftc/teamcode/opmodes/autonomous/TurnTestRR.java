package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "TurnTestRR", group = "Test")
public class TurnTestRR extends LinearOpMode {

    // Start pose in inches + radians
    private static final Pose2d INITIAL_POSE = new Pose2d(0.0, 0.0, 0.0);

    // Small forward move (inches)
    private static final double PREMOVE_FORWARD_IN = 2.0;

    // Turn controller tuning
    private static final double TURN_TOL_DEG = 2.0;        // stop when within this
    private static final double TURN_TIMEOUT_SEC = 3.0;    // safety timeout per turn

    // omega is radians/sec in RR
    private static final double MAX_OMEGA_RAD_PER_SEC = 3.0;
    private static final double MIN_OMEGA_RAD_PER_SEC = 0.6; // overcome static friction
    private static final double kP_TURN = 4.0;               // rad/sec per rad error

    @Override
    public void runOpMode() {
        PinpointDrive drive = new PinpointDrive(hardwareMap, INITIAL_POSE);

        telemetry.addLine("TurnTestRR init done");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Small forward move (relative)
        Action preMove = drive.actionBuilder(drive.pose)
                .lineToX(drive.pose.position.x + PREMOVE_FORWARD_IN)
                .build();
        Actions.runBlocking(preMove);

        // Turn tests (absolute headings in degrees)
        spinToHeadingDeg(drive, 90);
        sleep(250);
        spinToHeadingDeg(drive, 0);
        sleep(250);
        spinToHeadingDeg(drive, -90);
        sleep(250);
        spinToHeadingDeg(drive, 180);

        telemetry.addLine("DONE");
        telemetry.update();
        sleep(2000);
    }

    /**
     * Spins in place until current heading matches target heading (deg).
     * Commands vx=0, vy=0, omega only.
     */
    private void spinToHeadingDeg(PinpointDrive drive, double targetDeg) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double targetRad = Math.toRadians(targetDeg);

        while (opModeIsActive() && timer.seconds() < TURN_TIMEOUT_SEC) {
            drive.updatePoseEstimate();

            double currentRad = drive.pose.heading.toDouble();
            double errRad = normalizeRadians(targetRad - currentRad);

            double errDeg = Math.toDegrees(errRad);
            if (Math.abs(errDeg) <= TURN_TOL_DEG) break;

            // P control on heading error -> omega command
            double omega = kP_TURN * errRad;
            omega = Range.clip(omega, -MAX_OMEGA_RAD_PER_SEC, MAX_OMEGA_RAD_PER_SEC);

            // Ensure minimum turning power if still far away
            if (Math.abs(omega) < MIN_OMEGA_RAD_PER_SEC) {
                omega = Math.copySign(MIN_OMEGA_RAD_PER_SEC, omega);
            }

            // Spin in place: no translation
           drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), omega));

            telemetry.addLine("TURNING");
            telemetry.addData("Target (deg)", "%.1f", targetDeg);
            telemetry.addData("Current (deg)", "%.1f", Math.toDegrees(currentRad));
            telemetry.addData("Error (deg)", "%.1f", errDeg);
            telemetry.addData("Omega (rad/s)", "%.2f", omega);
            telemetry.update();
        }

        // Stop
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
        drive.updatePoseEstimate();

        telemetry.addLine("TURN DONE");
        telemetry.addData("Target (deg)", "%.1f", targetDeg);
        telemetry.addData("Final (deg)", "%.1f", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();
    }

    // Wrap to [-pi, pi]
    private static double normalizeRadians(double r) {
        while (r > Math.PI) r -= 2.0 * Math.PI;
        while (r < -Math.PI) r += 2.0 * Math.PI;
        return r;
    }
}
