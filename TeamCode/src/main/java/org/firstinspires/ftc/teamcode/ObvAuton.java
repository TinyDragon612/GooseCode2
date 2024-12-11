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
                .strafeToConstantHeading(new Vector2d(-10, -33))
                //.afterTime(3, robot.highChamber())
                .waitSeconds(1)
                .lineToY(-37)
                .splineToLinearHeading(new Pose2d(35, -50, 0), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(45, -10), 0)
                .turnTo(Math.toRadians(-100))
                .strafeTo(new Vector2d(45, -60))
                .strafeTo(new Vector2d(45, -10))
                .strafeTo(new Vector2d(58, -10))
                .strafeTo(new Vector2d(58, -70))
                .endTrajectory()
                .splineToConstantHeading(new Vector2d(42, -67), Math.toRadians(90))
                .lineToY(-63)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(0, -33), Math.toRadians(270))
                .lineToY(-37)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(55, -67), Math.toRadians(90))
                .lineToY(-63)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(5, -33), Math.toRadians(270))
                .lineToY(-37)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(55, -67), Math.toRadians(90));

        TrajectoryActionBuilder traj2 = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(180))
                .waitSeconds(0.5)
                .turn(Math.toRadians(180))
                .waitSeconds(0.5)
                .turn(Math.toRadians(180))
                .waitSeconds(0.5)
                .turn(Math.toRadians(180))
                .waitSeconds(0.5)
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
                            trajectoryActionCloseOut
                            //robot.highChamber()
                    )
            );
        }

    }


}
