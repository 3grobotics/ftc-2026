package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-44, 53, Math.toRadians(61)))
                // first shot
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(12, 24, Math.toRadians(90)), Math.toRadians(90))
                // first pickup
                .splineToSplineHeading(new Pose2d(12, 52, Math.toRadians(90)), Math.toRadians(90))
                // flush
                        .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(5, 55, Math.toRadians(90)), Math.toRadians(90))



                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}