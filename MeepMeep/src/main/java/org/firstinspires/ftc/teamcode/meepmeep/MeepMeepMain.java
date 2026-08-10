package org.firstinspires.ftc.teamcode.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public final class MeepMeepMain {
    private static final double IN_PER_TICK = 1.0;
    private static final double TRACK_WIDTH_TICKS = 12.698491025795713;

    private static final double MAX_WHEEL_VEL = 60.0;
    private static final double MAX_PROFILE_ACCEL = 50.0;

    private static final double MAX_ANG_VEL = Math.PI;
    private static final double MAX_ANG_ACCEL = Math.PI;

    private MeepMeepMain() {
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double trackWidth = IN_PER_TICK * TRACK_WIDTH_TICKS;

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(
                        MAX_WHEEL_VEL,
                        MAX_PROFILE_ACCEL,
                        MAX_ANG_VEL,
                        MAX_ANG_ACCEL,
                        trackWidth
                )
                .build();

        Pose2d startPose = new Pose2d(0.0, 0.0, 0.0);
        TrajectoryActionBuilder actionBuilder = bot.getDrive()
                .actionBuilder(startPose)
                .splineTo(new Vector2d(30.0, 30.0), Math.PI / 2.0)
                .splineTo(new Vector2d(0.0, 60.0), Math.PI);

        bot.runAction(actionBuilder.build());

        meepMeep
                .setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}
