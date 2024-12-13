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
@Autonomous(name = "AutonMin", group = "drive")
public class AutonMin extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, -70, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        RRActions robot = new RRActions(hardwareMap);



        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .afterTime(0, robot.highChamber())
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(0, -40))
                .afterTime(1.5, robot.scoreChamber())
                .afterTime(1.7, robot.openBack());


        TrajectoryActionBuilder traj2 = drive.actionBuilder(initialPose)
                .lineToY(-50)
                .afterTime(0.5, robot.slidesDown())
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .afterTime(1.5, robot.wall())
                .strafeToConstantHeading(new Vector2d(75, -75))
                .afterTime(1.7, robot.closeBack())
                .afterTime(2, robot.wallBump())
                .waitSeconds(2)
                .strafeToConstantHeading(new Vector2d(2, -65))
                .turn(Math.toRadians(90));

//                .lineToY(-50)
//                .afterTime(2.2, robot.slidesDown())
//                .waitSeconds(1)
//                .turn(Math.toRadians(180))
//                .afterTime(3.2, robot.wall())
//                .strafeToConstantHeading(new Vector2d(45, -85))
//                .afterTime(3.4, robot.closeBack())
//                .afterTime(3.7, robot.scoreChamber())
//                .waitSeconds(1)
//                .strafeToConstantHeading(new Vector2d(0, -60))
//                .afterTime(4.2, robot.slidesDown())
//                .turn(Math.toRadians(180));

        TrajectoryActionBuilder traj3 = drive.actionBuilder(initialPose)
                .afterTime(0, robot.highChamber())
                .waitSeconds(1.5)
                .strafeToConstantHeading(new Vector2d(0, -40))
                .afterTime(1.5, robot.scoreChamber())
                .afterTime(1.7, robot.openBack());

        TrajectoryActionBuilder traj4 = drive.actionBuilder(initialPose)
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(50, -75))
                .afterTime(0.5, robot.slidesDown())
                .waitSeconds(10);
//


        Action CloseOut1 = traj1.endTrajectory().fresh().build();

        Action CloseOut2 = traj2.endTrajectory().fresh().build();

        Action CloseOut3 = traj3.endTrajectory().fresh().build();
        Action CloseOut4 = traj4.endTrajectory().fresh().build();


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
                                CloseOut2,
                                traj3.build(),
                                CloseOut3,
                                traj4.build(),
                                CloseOut4
                        )
                );
            }
        }

    }


}
