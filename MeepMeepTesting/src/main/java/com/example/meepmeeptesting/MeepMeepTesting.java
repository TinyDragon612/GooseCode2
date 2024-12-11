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

        /*

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


                Pose2d initialPose = new Pose2d(-34, -62, Math.toRadians(180));


        TrajectoryActionBuilder path1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-52, -54), Math.toRadians(-140));

        TrajectoryActionBuilder path2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-48, -39), Math.toRadians(90));

        TrajectoryActionBuilder path3 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-52, -54), Math.toRadians(-140));

        TrajectoryActionBuilder path4 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-58, -39), Math.toRadians(90));

        TrajectoryActionBuilder path5 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-52, -54), Math.toRadians(-140));

        TrajectoryActionBuilder path6 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-55, -39), Math.toRadians(135));

        TrajectoryActionBuilder path7 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-52, -54), Math.toRadians(-140));

        TrajectoryActionBuilder path8 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-52, -40))
                .splineToLinearHeading(new Pose2d(-25, -5, Math.toRadians(0)), 0);

         */

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -70, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(-10, -33))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(35, -30, 0), Math.toRadians(-270))
                .splineToConstantHeading(new Vector2d(45, -10), 0)
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(45, -60))
                .strafeTo(new Vector2d(45, -10))
                .strafeTo(new Vector2d(58, -10))
                .strafeTo(new Vector2d(58, -70))
                .endTrajectory()
                .splineToConstantHeading(new Vector2d(42, -67), Math.toRadians(90))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(0, -33), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(55, -67), Math.toRadians(90))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(5, -33), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(55, -67), Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}