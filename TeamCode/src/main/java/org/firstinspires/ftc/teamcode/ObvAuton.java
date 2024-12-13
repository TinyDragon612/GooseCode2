package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "ObvAuton", group = "drive")
public class ObvAuton extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, -70, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        RRActions robot = new RRActions(hardwareMap);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-10, -40))
                //.afterTime(3, robot.highChamber())
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(35, -50, 0), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(45, -10), 0)
                .turn(Math.toRadians(105))
                .strafeTo(new Vector2d(45, -70))
                .strafeTo(new Vector2d(45, -10))
                .strafeTo(new Vector2d(53, -10))
                .strafeTo(new Vector2d(53, -70))
                .strafeTo(new Vector2d(47, -70))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(0, -60), Math.toRadians(285))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(55, -70), Math.toRadians(105))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(0, -60), Math.toRadians(285))
                .waitSeconds(1)
                .strafeTo(new Vector2d(55, -70));

        TrajectoryActionBuilder traj612 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-10, -40))
                .waitSeconds(1)
                .strafeTo(new Vector2d(35, -60))
                .splineToConstantHeading(new Vector2d(45, -10), 0)
                .turn(Math.toRadians(105))
                .waitSeconds(0.5)
                .turn(Math.toRadians(105))
                .strafeTo(new Vector2d(45, -70))
                .strafeTo(new Vector2d(45, -10))
                .strafeTo(new Vector2d(53, -10))
                .strafeTo(new Vector2d(53, -70))
                //.endTrajectory()
                //.splineToConstantHeading(new Vector2d(50, -70), Math.toRadians(90))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-5, -60))
                .turn(Math.toRadians(230))
                .lineToY(-62)
                .waitSeconds(1)
                .strafeTo(new Vector2d(0, -50))
                .turn(Math.toRadians(230))
                .strafeTo(new Vector2d(55, -70))
                .waitSeconds(1)
                .strafeTo(new Vector2d(0, -50))
                .turn(Math.toRadians(230))
                .strafeTo(new Vector2d(0, -40))
                .waitSeconds(1)
                .strafeTo(new Vector2d(55, -70));

        TrajectoryActionBuilder traj2 = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(180))
                .waitSeconds(1)
                .turn(Math.toRadians(180))
                .waitSeconds(1)
                .turn(Math.toRadians(180))
                .waitSeconds(1)
                .turn(Math.toRadians(180))
                .waitSeconds(1)
                .turn(Math.toRadians(180));

        TrajectoryActionBuilder traj3 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(10, -45));


        Action trajectoryActionCloseOut = traj1.endTrajectory().fresh().build();


        //Actions.runBlocking(robot.highChamber());


        while(opModeInInit()){
            telemetry.addData("initialized", "yay");
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            Actions.runBlocking(
                    new SequentialAction(
                            traj1.build(),

                            trajectoryActionCloseOut,
                            robot.highChamber()
                    )
            );
        }

    }


}
