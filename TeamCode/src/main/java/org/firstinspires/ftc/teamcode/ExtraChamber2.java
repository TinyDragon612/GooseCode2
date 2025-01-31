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

@Autonomous(name = "Swivolio", group = "Auton")
public class ExtraChamber2 extends OpMode {

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

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 70, Math.toRadians(180));
    private final Pose scorePrePose = new Pose(39,75, Math.toRadians(180));
    private final Pose grabFirstSample = new Pose(10, 12);
    private final Pose grabFirstSample2 = new Pose(16.5, 12);
    private final Pose turnHelper1 = new Pose(15, 8, Math.toRadians((135)));
    private final Pose grabSecondSample = new Pose(16.5, 2);
    private final Pose turnHelper2 = new Pose(21, 8);
    private final Pose grabForwardPose = new Pose(-5, 24);
    private final Pose grabPose = new Pose(5, 24, Math.toRadians(0));
    private final Pose scoreFirstPose = new Pose(40, 72, Math.toRadians(180));
    private final Pose scoreSecondPose = new Pose(40, 70, Math.toRadians(180));
    private final Pose safetyScore = new Pose(37, 69, Math.toRadians(180));
    private final Pose scoreThirdPose = new Pose(40, 67, Math.toRadians(180));
    private final Pose parkPose = new Pose(8, 24);
    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;
    private PathChain grabSpline1, scoreFirst, grabSecond, grabSpline3, grabSpline4, grabFirstSpecimen, scoreSecond, grabThird, scoreThird, park, grabSpline2;
    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePrePose)));
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(180));
        grabSpline1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePrePose), new Point(grabFirstSample)))
                .setLinearHeadingInterpolation(scorePrePose.getHeading(), grabFirstSample.getHeading())
                .addPath(new BezierLine(new Point(grabFirstSample), new Point(grabFirstSample2)))
                .setConstantHeadingInterpolation(Math.toRadians((0)))
                .build();
        grabSpline2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabFirstSample2), new Point(turnHelper1)))
                .setLinearHeadingInterpolation(grabFirstSample2.getHeading(), turnHelper1.getHeading())
                .build();
        grabSpline3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(turnHelper1), new Point(grabSecondSample)))
                .setConstantHeadingInterpolation(Math.toRadians((0)))
                .build();
        grabSpline4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabSecondSample), new Point(turnHelper2)))
                .setConstantHeadingInterpolation(Math.toRadians((150)))
                .build();
        grabFirstSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(turnHelper2), new Point(grabPose)))
                .setConstantHeadingInterpolation(Math.toRadians((0)))
                .addPath(new BezierLine(new Point(grabPose), new Point(grabForwardPose)))
                .setConstantHeadingInterpolation(Math.toRadians((0)))
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
                slides.setTargetPosition(1800);
                if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                    follower.followPath(scorePreload, true);
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy() && slides.getCurrentLeftPosition() > slides.targetPosition - 10) {
                    slides.setTargetPosition(1500);
                    setPathState(2);
                }
                break;
            case 2:
                if(slides.getCurrentLeftPosition() < 1550){
                    openBack();
                    slides.setTargetPosition(0);
                    follower.followPath(grabSpline1, false);
                    setPathState(3);
                }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 3){
                    extendR.setPosition(0.950);
                    extendL.setPosition(0.050);
                    if (pathTimer.getElapsedTimeSeconds() > 4){
                        frontR.setPosition(0.65);
                        frontL.setPosition(0.40);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                follower.followPath(grabSpline2, true);
                setPathState(5);
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 3){
                    frontR.setPosition(0);
                    frontL.setPosition(1);
                }
                if(frontR.getPosition() == 0){
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 2){
                    follower.followPath(grabSpline3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds() > 1){
                    frontR.setPosition(0.65);
                    frontL.setPosition(0.40);
                    setPathState(8);
                }
                break;
            case 8:
                follower.followPath(grabSpline4, true);
                setPathState(9);
                break;
            case 9:
                if(pathTimer.getElapsedTimeSeconds() > 0){
                    frontR.setPosition(1);
                    frontL.setPosition(0);
                }
                if(pathTimer.getElapsedTimeSeconds() > 0.5){
                    extendR.setPosition(1);
                    extendL.setPosition(0);

                    slides.setTargetPosition(455);
                }
                if(pathTimer.getElapsedTimeSeconds() > 3 && extendR.getPosition() == 1){
                    follower.followPath(grabFirstSpecimen, true);
                    setPathState(10);
                }
                break;
            case 10:
                if(pathTimer.getElapsedTimeSeconds() > 2 && slides.getCurrentLeftPosition() > slides.targetPosition - 10){
                    closeBack();
                    slides.setTargetPosition(1800);
                    setPathState(11);
                }
                break;
            case 11:
                if(slides.getCurrentRightPosition() > 700){
                    follower.followPath(scoreFirst, true);
                    setPathState(12);
                }
                break;
            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && slides.getCurrentLeftPosition() > slides.targetPosition - 10) {
                    slides.setTargetPosition(1500);
                    setPathState(13);
                }
                break;
            case 13:
                if(slides.getCurrentLeftPosition() < 1550){
                    openBack();
                    slides.setTargetPosition(450);
                    follower.followPath(grabSecond, true);
                    setPathState(14);
                }
                break;
            case 14:
                if(pathTimer.getElapsedTimeSeconds() > 3 && slides.getCurrentLeftPosition() > slides.targetPosition - 10){
                    closeBack();
                    slides.setTargetPosition(1800);
                    setPathState(15);
                }
                break;
            case 15:
                if(slides.getCurrentRightPosition() > 700){
                    follower.followPath(scoreSecond, true);
                    setPathState(16);
                }
                break;
            case 16:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && slides.getCurrentLeftPosition() > slides.targetPosition - 10) {
                    slides.setTargetPosition(1500);
                    setPathState(17);
                }
                break;
            case 17:
                if(slides.getCurrentLeftPosition() < 1550){
                    openBack();
                    slides.setTargetPosition(450);
                    follower.followPath(grabThird, true);
                    setPathState(18);
                }
                break;
            case 18:
                if(pathTimer.getElapsedTimeSeconds() > 3 && slides.getCurrentLeftPosition() > slides.targetPosition - 10){
                    closeBack();
                    slides.setTargetPosition(1800);
                    setPathState(19);
                }
                break;
            case 19:
                if(slides.getCurrentRightPosition() > 700){
                    follower.followPath(scoreThird, true);
                    setPathState(20);
                }
                break;
            case 20:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && slides.getCurrentLeftPosition() > slides.targetPosition - 10) {
                    slides.setTargetPosition(1500);
                    setPathState(21);
                }
                break;
            case 21:
                if(slides.getCurrentLeftPosition() < 1550){
                    openBack();
                    slides.setTargetPosition(0);
                    follower.followPath(park, false);
                    setPathState(22);
                }
                break;
            case 22:
                if(!follower.isBusy()){
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
    public void init_loop() {}

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