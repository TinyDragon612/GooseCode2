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
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -70, Math.toRadians(-90)))
                .lineToY(-33)
                .waitSeconds(2)
                .strafeTo(new Vector2d(35, -40))
                .endTrajectory()
                .lineToY(-10)
                .strafeTo(new Vector2d(45, -10))
                .strafeTo(new Vector2d(45, -60))
                .strafeTo(new Vector2d(45, -10))
                .strafeToLinearHeading(new Vector2d(55, -10), Math.toRadians(-180))
                .strafeTo(new Vector2d(55, -60))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(0, -33), Math.toRadians(-90))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(55, -60), Math.toRadians(-180))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(0, -33), Math.toRadians(-90))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(55, -60), Math.toRadians(-180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}