package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingFar {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61, 25, Math.toRadians(0)))
                        // first pickup
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(61, 60, Math.toRadians(90)), Math.toRadians(90))
                // first drive back
                .strafeTo(new Vector2d(61,25))

                // second pickup
                .strafeTo(new Vector2d(61,60))

                // second drive back
                .strafeTo(new Vector2d(61,25))

                // third pickup
                .strafeTo(new Vector2d(61,60))

                // third drive back
                .strafeTo(new Vector2d(61,25))

                // fourth pickup
                .strafeTo(new Vector2d(61,60))

                // fourth drive back
                .strafeTo(new Vector2d(61,25))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}