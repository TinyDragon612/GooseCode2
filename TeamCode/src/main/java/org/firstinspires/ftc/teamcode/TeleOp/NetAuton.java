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

import org.firstinspires.ftc.teamcode.RRLegacy.MecanumDrive;
import org.firstinspires.ftc.teamcode.RRLegacy.RRActions;

@Config
@Autonomous(name = "NetAuton", group = "drive")
public class NetAuton extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-40, -70, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        RRActions robot = new RRActions(hardwareMap);



        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .waitSeconds(9) // change
                .afterTime(0, robot.highChamber())
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(-10, -32))
                .afterTime(1.5, robot.scoreChamber())
                .afterTime(1.7, robot.openBack());


        TrajectoryActionBuilder traj2 = drive.actionBuilder(initialPose)
                .lineToY(-50)
                .afterTime(0.5, robot.slidesDown())
                .waitSeconds(1.25)
                .strafeToConstantHeading(new Vector2d(-45, -50))
                .turn(Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(-45, -20))
                .afterTime(3, robot.slideHang())
                .strafeToConstantHeading(new Vector2d(-10, -20))
                .waitSeconds(20);
                //.afterTime(3, robot.setHang());



        Action CloseOut1 = traj1.endTrajectory().fresh().build();

        Action CloseOut2 = traj2.endTrajectory().fresh().build();

        //Action CloseOut3 = traj3.endTrajectory().fresh().build();


        Actions.runBlocking(robot.closeBack());


        while(opModeInInit()){
            telemetry.addData("initialized", "yay");
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            if(opModeIsActive()){
                Actions.runBlocking(
                        new SequentialAction(
                                traj1.build(),
                                CloseOut1,
                                traj2.build(),
                                CloseOut2
                                //traj3.build(),
                                //CloseOut3
                        )
                );
            }
        }

    }


}
