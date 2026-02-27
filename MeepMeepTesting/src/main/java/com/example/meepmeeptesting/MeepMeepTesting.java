// File: MeepMeepTesting/src/main/java/com/example/meepmeeptesting/MeepMeepTesting.java
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Rotation2d;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

/**
 * Goal:
 * - Keep your OpMode unchanged (local coordinates: start=(0,0,0)).
 * - Still visualize on the official field background in MeepMeep.
 *
 * How:
 * - Choose FIELD_START (where the robot really starts on the field in MeepMeep coordinates).
 * - Convert every LOCAL waypoint (your OpMode numbers) into FIELD coordinates for MeepMeep visualization.
 *
 * Versions:
 * - RR 1.0.1: Pose2d(Vector2d, Rotation2d), radians.
 * - MeepMeep 0.1.7: compatible with actionBuilder.
 * - Pinpoint/OTOS repo affects robot localization, not MeepMeep.
 */
public class MeepMeepTesting {

    // =========================
    // 1) YOUR LOCAL (OpMode) POSES (UNCHANGED)
    // =========================
    private static final Pose2d LOCAL_START =
            new Pose2d(new Vector2d(0.0, 0.0), Rotation2d.exp(0.0));

    private static final Pose2d LOCAL_SHOOT =
            new Pose2d(new Vector2d(-52.3, 0.0), Rotation2d.exp(Math.toRadians(0.0)));

    private static final Pose2d LOCAL_TARGET_1 =
            new Pose2d(new Vector2d(-44.0, -14.3), Rotation2d.exp(Math.toRadians(140.0)));
    private static final double HEADING_OFFSET = Math.PI;
    private static final Pose2d LOCAL_PICKUP_END_1 =
            new Pose2d(new Vector2d(-23.3, -30.6), Rotation2d.exp(Math.toRadians(140.0)));

    private static final Pose2d LOCAL_PICKUP_2 =
            new Pose2d(new Vector2d(-58.7, -34.5), Rotation2d.exp(Math.toRadians(140.0)));

    private static final Pose2d LOCAL_PICKUP_END_2 =
            new Pose2d(new Vector2d(-41.5, -47.6), Rotation2d.exp(Math.toRadians(140.0)));

    private static final Pose2d LOCAL_PICKUP_3 =
            new Pose2d(new Vector2d(-76.7, -45.45), Rotation2d.exp(Math.toRadians(140.0)));

    private static final Pose2d LOCAL_PICKUP_END_3 =
            new Pose2d(new Vector2d(-49.0, -71.0), Rotation2d.exp(Math.toRadians(140.0)));

    private static final Pose2d LOCAL_FINAL =
            new Pose2d(new Vector2d(-42.4, 8.6), Rotation2d.exp(Math.toRadians(0.0)));

    // =========================
    // 2) SET THIS ONCE: REAL FIELD START (MeepMeep coords)
    // =========================
    // You MUST tune these 3 numbers so the robot appears exactly where you start on the field background.
    //
    // If you want "facing the origin (0,0) from (-50,50)" then heading is -45 degrees.
    // Change sx/sy to match your exact starting tile position.
    private static final Pose2d FIELD_START =
            new Pose2d(new Vector2d(-50.0, 50.0), Rotation2d.exp(Math.toRadians(-45.0)));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // Convert LOCAL waypoints into FIELD waypoints for MeepMeep visualization
        Pose2d FIELD_SHOOT        = localToField(LOCAL_SHOOT, FIELD_START);
        Pose2d FIELD_PICKUP_3     = localToField(LOCAL_PICKUP_3, FIELD_START);
        Pose2d FIELD_PICKUP_END_3 = localToField(LOCAL_PICKUP_END_3, FIELD_START);
        Pose2d FIELD_TARGET_1     = localToField(LOCAL_TARGET_1, FIELD_START);
        Pose2d FIELD_PICKUP_END_1 = localToField(LOCAL_PICKUP_END_1, FIELD_START);
        Pose2d FIELD_FINAL        = localToField(LOCAL_FINAL, FIELD_START);

        myBot.runAction(
                // IMPORTANT: start the sim at FIELD_START so the bot spawns in the right place on the map
                myBot.getDrive().actionBuilder(FIELD_START)

                        // Pre-move (local lineToX becomes a field lineToX of the transformed X)
                        .lineToX(FIELD_SHOOT.position.x)

                        // Full sequence (mirrors your OpMode structure, but in FIELD coords)
                        .splineToLinearHeading(
                                FIELD_PICKUP_3,
                                Math.atan2(
                                        FIELD_PICKUP_3.position.y - FIELD_SHOOT.position.y,
                                        FIELD_PICKUP_3.position.x - FIELD_SHOOT.position.x
                                )
                        )
                        .strafeTo(new Vector2d(
                                FIELD_PICKUP_END_3.position.x,
                                FIELD_PICKUP_END_3.position.y
                        ))


                        .splineToLinearHeading(FIELD_SHOOT, Math.toRadians(0.0))

                        .splineToLinearHeading(FIELD_TARGET_1, Math.toRadians(0.0))
                        .splineToLinearHeading(
                                FIELD_TARGET_1,
                                Math.atan2(
                                        FIELD_TARGET_1.position.y - FIELD_SHOOT.position.y,
                                        FIELD_TARGET_1.position.x - FIELD_SHOOT.position.x
                                )
                        )
                        .strafeTo(new Vector2d(
                                FIELD_PICKUP_END_1.position.x,
                                FIELD_PICKUP_END_1.position.y
                        ))



                        .splineToLinearHeading(FIELD_SHOOT, Math.toRadians(0.0))

                        .splineToLinearHeading(FIELD_FINAL, Math.toRadians(0.0))



                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    /**
     * Convert a pose expressed in LOCAL (OpMode) coordinates into FIELD coordinates for MeepMeep display.
     *
     * LOCAL frame definition (your OpMode):
     * - origin at robot start
     * - heading 0 is your robot's initial facing direction
     *
     * FIELD frame definition (MeepMeep):
     * - origin at field center (per the background)
     *
     * fieldPose = fieldStart ⊕ localPose
     */
    private static Pose2d localToField(Pose2d local, Pose2d fieldStart) {
        // Apply heading offset so local axes match your OpMode convention
        double sh = fieldStart.heading.toDouble() + HEADING_OFFSET;

        double c = Math.cos(sh);
        double s = Math.sin(sh);

        double lx = local.position.x;
        double ly = local.position.y;

        double fx = fieldStart.position.x + (lx * c - ly * s);
        double fy = fieldStart.position.y + (lx * s + ly * c);

        double fh = wrapRadians(local.heading.toDouble() + sh);

        return new Pose2d(new Vector2d(fx, fy), Rotation2d.exp(fh));
    }

    private static double wrapRadians(double a) {
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        while (a >   Math.PI) a -= 2.0 * Math.PI;
        return a;
    }
}
