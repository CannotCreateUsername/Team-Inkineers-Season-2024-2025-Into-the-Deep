package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(32, -0, 0))
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(38, -52, 0), -Math.PI/2)
//                .waitSeconds(1)
//                .setTangent(Math.PI/2)
//                .splineToConstantHeading(new Vector2d(0, -34), Math.PI/2)
//                .setTangent(-Math.PI)
//                .splineToConstantHeading(new Vector2d(18, -52), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToConstantHeading(new Vector2d(50, -12), 0)
//                .setTangent(Math.PI/2)
//                .splineToConstantHeading(new Vector2d(38, -52), Math.PI/2)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}