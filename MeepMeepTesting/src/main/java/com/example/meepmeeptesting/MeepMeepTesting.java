// File: MeepMeepTesting/src/main/java/com/example/meepmeeptesting/MeepMeepTesting.java
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Rotation2d;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    // ======= FIELD start (MeepMeep frame) =======
    private static final double START_X = -60.0;
    private static final double START_Y =  56.0;

    // Opposite corner assumption (adjust if needed)
    private static final double OPP_X =  72.0;
    private static final double OPP_Y = -72.0;

    // Start heading as radians, then convert to Rotation2d
    private static final double START_HEADING_RAD =
            Math.atan2(OPP_Y - START_Y, OPP_X - START_X);

    private static final Rotation2d START_HEADING = Rotation2d.exp(START_HEADING_RAD);

    // ======= LOCAL poses (your auton numbers, expressed in LOCAL frame) =======
    // Build them using Vector2d + Rotation2d so we match MeepMeep’s Pose2d type.
    private static final Pose2d SHOOTING_POSITION_L =
            new Pose2d(new Vector2d(-52.3, 0.0), Rotation2d.exp(Math.toRadians(0)));

    private static final Pose2d TARGET_1_L =
            new Pose2d(new Vector2d(-44.0, -14.3), Rotation2d.exp(Math.toRadians(140)));

    private static final Pose2d PICKUP_END_1_L =
            new Pose2d(new Vector2d(-23.3, -30.6), Rotation2d.exp(Math.toRadians(140)));

    private static final Pose2d PICKUP_3_L =
            new Pose2d(new Vector2d(-76.7, -50.45), Rotation2d.exp(Math.toRadians(140)));

    private static final Pose2d PICKUP_END_3_L =
            new Pose2d(new Vector2d(-49.0, -71.0), Rotation2d.exp(Math.toRadians(140)));

    private static final Pose2d FINAL_L =
            new Pose2d(new Vector2d(-42.4, 8.6), Rotation2d.exp(Math.toRadians(0)));

    // Convert a LOCAL pose into a FIELD pose (rotate + translate + rotate heading)
    private static Pose2d toField(Pose2d local) {
        double lx = local.position.x;
        double ly = local.position.y;

        double c = Math.cos(START_HEADING_RAD);
        double s = Math.sin(START_HEADING_RAD);

        double fx = START_X + (lx * c - ly * s);
        double fy = START_Y + (lx * s + ly * c);

        // heading: fieldHeading = startHeading * localHeading
        Rotation2d fieldHeading = START_HEADING.times(local.heading);

        return new Pose2d(new Vector2d(fx, fy), fieldHeading);
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d startFieldPose = new Pose2d(new Vector2d(START_X, START_Y), START_HEADING);

        Pose2d SHOOTING_POSITION = toField(SHOOTING_POSITION_L);
        Pose2d PICKUP_3          = toField(PICKUP_3_L);
        Pose2d PICKUP_END_3      = toField(PICKUP_END_3_L);
        Pose2d TARGET_1          = toField(TARGET_1_L);
        Pose2d PICKUP_END_1      = toField(PICKUP_END_1_L);
        Pose2d FINAL             = toField(FINAL_L);

        // Drivetrain-only sequencing (avoid lineToY* to prevent tangent exceptions)
        myBot.runAction(
                myBot.getDrive().actionBuilder(startFieldPose)

                        .splineToLinearHeading(SHOOTING_POSITION, START_HEADING_RAD)

                        .splineToLinearHeading(PICKUP_3, START_HEADING_RAD)
                        .splineToLinearHeading(PICKUP_END_3, START_HEADING_RAD)

                        .splineToLinearHeading(SHOOTING_POSITION, START_HEADING_RAD)

                        .splineToLinearHeading(TARGET_1, START_HEADING_RAD)
                        .splineToLinearHeading(PICKUP_END_1, START_HEADING_RAD)

                        .splineToLinearHeading(SHOOTING_POSITION, START_HEADING_RAD)
                        .splineToLinearHeading(FINAL, START_HEADING_RAD)

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
