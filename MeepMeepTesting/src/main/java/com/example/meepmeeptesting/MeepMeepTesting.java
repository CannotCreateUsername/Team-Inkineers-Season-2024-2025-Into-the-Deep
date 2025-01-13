package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.time.Year;

public class MeepMeepTesting {
    private static final double X_OFFSET = 12;
    private static final double Y_OFFSET = -64;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(X_OFFSET, Y_OFFSET, 0))
                .strafeToLinearHeading(new Vector2d(-6 + X_OFFSET, 36 + Y_OFFSET), 0)
                .setTangent(-Math.PI/2)
                .splineToConstantHeading(new Vector2d(33 + X_OFFSET, 26 + Y_OFFSET), 0)
                .strafeToConstantHeading(new Vector2d(38 + X_OFFSET, 15 + Y_OFFSET))
                .waitSeconds(0.5)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(43.5 + X_OFFSET, 27.5 + Y_OFFSET), Math.PI/2)
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(28 + X_OFFSET, -3 + Y_OFFSET), -Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(-8 + X_OFFSET, 36 + Y_OFFSET), Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToConstantHeading(new Vector2d(28 + X_OFFSET, -3 + Y_OFFSET), -Math.PI/2)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}