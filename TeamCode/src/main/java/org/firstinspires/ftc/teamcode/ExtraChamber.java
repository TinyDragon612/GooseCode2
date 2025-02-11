package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.PPTuning.LConstants;
import org.firstinspires.ftc.teamcode.PPTuning.FConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Chambolio", group = "Auton")
public class ExtraChamber extends OpMode {

    private ServoImplEx backR;
    private ServoImplEx backL;
    private ServoImplEx frontR;
    private ServoImplEx frontL;
    private ServoImplEx extendR;
    private ServoImplEx extendL;
    public MotorMech2 slides;
    public DcMotorEx hang;


    private double cranePower = 0.1;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 70, Math.toRadians(180));
    private final Pose scorePrePose = new Pose(39,75, Math.toRadians(180)); //x=39
    private final Pose pushSplineControl1 = new Pose(10.6, 35);
    private final Pose pushSplineEnd = new Pose(20, 26, Math.toRadians(0));
    private final Pose returnFirst = new Pose(45,26);
    private final Pose strafeFirst = new Pose(45, 15);
    private final Pose pushFirst = new Pose(16, 15);
    private final Pose returnSecond = new Pose(45, 15);
    private final Pose strafeSecond = new Pose(45, 3);
    private final Pose pushSecondControl = new Pose(155, 3);
    private final Pose pushSecond =  new Pose(10, 3);
    private final Pose grabSplineControl = new Pose(30, 25);
    private final Pose grabForwardPose = new Pose(-4, 24); //THIS ONE HERE IS WHEN ITS GRABBING OFF THE WALL I THINK
    private final Pose grabPose = new Pose(5, 24, Math.toRadians(0));
    private final Pose scoreFirstPose = new Pose(40, 72, Math.toRadians(180));
    private final Pose scoreSecondPose = new Pose(41, 69, Math.toRadians(180));
    private final Pose safetyScore = new Pose(37, 69, Math.toRadians(180));
    private final Pose scoreThirdPose = new Pose(40, 67, Math.toRadians(180));
    private final Pose parkPose = new Pose(5, 5);
    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;
    private PathChain pushSpline, pushBlocks, grabSpline, scoreFirst, grabSecond, scoreSecond, grabThird, scoreThird, park, parkGood;
    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePrePose)));
        scorePreload.setZeroPowerAccelerationMultiplier(7);
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(180));

        pushSpline = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePrePose), new Point(pushSplineControl1), new Point(pushSplineEnd)))
                .setLinearHeadingInterpolation(scorePrePose.getHeading(), pushSplineEnd.getHeading())
                .build();
        pushBlocks  = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushSplineEnd), new Point(returnFirst)))
                .setConstantHeadingInterpolation(0)
                .setPathEndTimeoutConstraint(20)
                .setPathEndTValueConstraint(0.95)
                .addPath(new BezierLine(new Point(returnFirst), new Point(strafeFirst)))
                .setPathEndTimeoutConstraint(20)
                .setPathEndTValueConstraint(0.95)
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(strafeFirst), new Point(pushFirst)))
                .setPathEndTimeoutConstraint(20)
                .setPathEndTValueConstraint(0.95)
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(pushFirst), new Point(returnSecond)))
                .setPathEndTimeoutConstraint(100)
                .setPathEndTValueConstraint(0.95)
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(returnSecond), new Point(strafeSecond)))
                .setPathEndTimeoutConstraint(100)
                .setPathEndTValueConstraint(0.95)
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(strafeSecond), new Point(pushSecond)))
                .setPathEndTimeoutConstraint(100)
                .setPathEndTValueConstraint(0.95)
                .setConstantHeadingInterpolation(0)
                .build();
        //from here on is new code
//                .addPath(new BezierCurve(new Point(pushFirst), new Point(pushSecondControl), new Point(pushSecond)))
//                .setConstantHeadingInterpolation(0)
//                .build();
        grabSpline = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pushSecond), new Point(grabSplineControl), new Point(grabForwardPose)))
                .setZeroPowerAccelerationMultiplier(8)
                .setConstantHeadingInterpolation(0)
                .build();
        scoreFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabForwardPose), new Point(scoreFirstPose)))
                .setLinearHeadingInterpolation(grabPose.getHeading(), scoreFirstPose.getHeading())
                .build();
        scoreSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabForwardPose), new Point(scoreSecondPose)))
                .setLinearHeadingInterpolation(grabPose.getHeading(), scoreSecondPose.getHeading())
                .build();
        scoreThird = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabForwardPose), new Point(scoreThirdPose)))
                .setLinearHeadingInterpolation(grabPose.getHeading(), scoreThirdPose.getHeading())
                .build();
        grabSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreFirstPose), new Point(grabPose)))
                .setLinearHeadingInterpolation(scoreFirstPose.getHeading(), grabPose.getHeading())
                .setPathEndTimeoutConstraint(500)
                .addPath(new BezierLine(new Point(grabPose), new Point(grabForwardPose)))
                .setZeroPowerAccelerationMultiplier(5)
                .setConstantHeadingInterpolation(0)
                .build();
        grabThird = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSecondPose), new Point(safetyScore)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(new Point(safetyScore), new Point(grabPose)))
                .setLinearHeadingInterpolation(safetyScore.getHeading(), grabPose.getHeading())
                .addPath(new BezierLine(new Point(grabPose), new Point(grabForwardPose)))
                .setConstantHeadingInterpolation(grabPose.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreThirdPose), new Point(safetyScore)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(new Point(safetyScore), new Point(parkPose)))
                .setConstantHeadingInterpolation(180)
                .build();
        parkGood = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabForwardPose), new Point(parkPose)))
                .build();



    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                slides.setTargetPosition(1850);
                extendL.setPosition(1);
                extendR.setPosition(0);
                if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                    follower.followPath(scorePreload, true);
                    setPathState(1);
                }
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && slides.getCurrentLeftPosition() > slides.targetPosition - 10) {
                    slides.setTargetPosition(1500);
                    setPathState(2);
                }
                break;
            case 2:
                if(slides.getCurrentLeftPosition() < 1550){
                    openBack();
                    slides.setTargetPosition(0);
                    follower.followPath(pushSpline, false);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    follower.followPath(pushBlocks, false);
                    if(pathTimer.getElapsedTimeSeconds() > 1){
                        slides.setTargetPosition(455);
                    }
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(grabSpline, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 3 && slides.getCurrentLeftPosition() > slides.targetPosition - 10){
                    closeBack();
                    slides.setTargetPosition(1850);
                    setPathState(6);
                }
                break;
            case 6:
                if(slides.getCurrentRightPosition() > 700){
                    follower.followPath(scoreFirst, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy() && slides.getCurrentLeftPosition() > slides.targetPosition - 10) {
                    slides.setTargetPosition(1500);
                    setPathState(8);
                }
                break;
            case 8:
                if(slides.getCurrentLeftPosition() < 1550 && pathTimer.getElapsedTimeSeconds() > 0.5){
                    openBack();
                    slides.setTargetPosition(455);
                    follower.followPath(grabSecond, true);
                    setPathState(9);
                }
                break;
            case 9:
                if(pathTimer.getElapsedTimeSeconds() > 3 && slides.getCurrentLeftPosition() > slides.targetPosition - 10){
                    closeBack();
                    slides.setTargetPosition(1850);
                    setPathState(10);
                }
                break;
            case 10:
                if(slides.getCurrentRightPosition() > 700){
                    follower.followPath(scoreSecond, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy() && slides.getCurrentLeftPosition() > slides.targetPosition - 10) {
                    slides.setTargetPosition(1500);
                    setPathState(16);
                }
                break;
            case 12:
                if(slides.getCurrentLeftPosition() < 1550 && pathTimer.getElapsedTimeSeconds() > 0.5){
                    openBack();
                    slides.setTargetPosition(450);
                    follower.followPath(grabThird, true);
                    setPathState(13);
                }
                break;
            case 13:
                if(pathTimer.getElapsedTimeSeconds() > 3 && slides.getCurrentLeftPosition() > slides.targetPosition - 10){
                    closeBack();
                    slides.setTargetPosition(600); //should be 1800 normally
                    setPathState(14);
                }
                break;
            case 14:
                if(slides.getCurrentRightPosition() > 700){
                    follower.followPath(scoreThird, true);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy() && slides.getCurrentLeftPosition() > slides.targetPosition - 10) {
                    slides.setTargetPosition(1500);
                    setPathState(16);
                }
                break;
            case 16:
                if(slides.getCurrentLeftPosition() < 1550 && pathTimer.getElapsedTimeSeconds() > 0.5){ //normally < 1550 // then 580
                    openBack();
                    slides.setTargetPosition(0);
                    follower.followPath(parkGood, false);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()){
                    if(pathTimer.getElapsedTimeSeconds() > 2){
                        slides.setPower(0);
                    }
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        slides = new MotorMech2(hardwareMap, cranePower, false);
        frontR = hardwareMap.get(ServoImplEx.class, "frontR");
        frontL = hardwareMap.get(ServoImplEx.class, "frontL");
        backR = hardwareMap.get(ServoImplEx.class, "backR");
        backL = hardwareMap.get(ServoImplEx.class, "backL");
        extendR = hardwareMap.get(ServoImplEx.class, "extendR");
        extendL = hardwareMap.get(ServoImplEx.class, "extendL");
        closeBack();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        telemetry.addData("Position", follower.getPose());
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    public void closeBack(){
        backR.setPosition(0.75);
        backL.setPosition(0.30);
    }
    public void openBack(){
        backR.setPosition(1);
        backL.setPosition(0);
    }
}