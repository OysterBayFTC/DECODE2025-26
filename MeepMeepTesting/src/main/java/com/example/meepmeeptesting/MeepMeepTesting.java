// File: MeepMeepTesting/src/main/java/com/example/meepmeeptesting/MeepMeepTesting.java
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Rotation2d;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    // Match Road Runner coordinates exactly (inches + radians).
    private static final Pose2d INITIAL_POSE =
            new Pose2d(new Vector2d(0.0, 0.0), Rotation2d.exp(0.0));

    private static final Pose2d SHOOTING_POSITION =
            new Pose2d(new Vector2d(-52.3, 0.0), Rotation2d.exp(Math.toRadians(0)));

    private static final Pose2d TARGET_1 =
            new Pose2d(new Vector2d(-44.0, -14.3), Rotation2d.exp(Math.toRadians(140)));

    private static final Pose2d PICKUP_END_1 =
            new Pose2d(new Vector2d(-23.3, -30.6), Rotation2d.exp(Math.toRadians(140)));

    private static final Pose2d PICKUP_3 =
            new Pose2d(new Vector2d(-76.7, -50.45), Rotation2d.exp(Math.toRadians(140)));

    private static final Pose2d PICKUP_END_3 =
            new Pose2d(new Vector2d(-49.0, -71.0), Rotation2d.exp(Math.toRadians(140)));

    private static final Pose2d FINAL =
            new Pose2d(new Vector2d(-42.4, 8.6), Rotation2d.exp(Math.toRadians(0)));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(INITIAL_POSE)
                        .lineToX(SHOOTING_POSITION.position.x)
                        .splineToLinearHeading(PICKUP_3, Math.toRadians(0))
                        .lineToX(PICKUP_END_3.position.x)
                        .lineToY(PICKUP_END_3.position.y)
                        .splineToLinearHeading(SHOOTING_POSITION, Math.toRadians(0))
                        .splineToLinearHeading(TARGET_1, Math.toRadians(0))
                        .lineToX(PICKUP_END_1.position.x)
                        .lineToY(PICKUP_END_1.position.y)
                        .splineToLinearHeading(SHOOTING_POSITION, Math.toRadians(0))
                        .splineToLinearHeading(FINAL, Math.toRadians(0))
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
