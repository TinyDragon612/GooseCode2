package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.MotorMech2;
//import org.firstinspires.ftc.teamcode.pedroPathing.configs.Subsystem;
import org.firstinspires.ftc.teamcode.PPTuning.LConstants;
import org.firstinspires.ftc.teamcode.PPTuning.FConstants;


@Autonomous(name = "Specimens grab with claw PEDRO", group = "Pedro")
public class specimengrab extends OpMode {

    private Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** This is our subsystem.
     * We call its methods to manipulate the stuff that it has within the subsystem. */
    private ServoImplEx backR;
    private ServoImplEx backL;
    private ServoImplEx frontR;
    private ServoImplEx frontL;
    private ServoImplEx extendR;
    private ServoImplEx extendL;
    public MotorMech2 slides;
    public DcMotorEx hang;

    private final Pose startPose = new Pose(9, 63, Math.toRadians(0));
    private final Pose score1Pose = new Pose(37, 63, Math.toRadians(0));
    private final Pose score2Pose = new Pose(37, 64, Math.toRadians(0));
    private final Pose score3Pose = new Pose(37, 65.5, Math.toRadians(0));
    private final Pose score4Pose = new Pose(37, 69, Math.toRadians(0));
    private final Pose score5Pose = new Pose(37, 73, Math.toRadians(0));


    /** Grabbing the specimen from the observation zone */
    private final Pose grabBackPose = new Pose(20, 32, Math.toRadians(0));
    private final Pose grabPose = new Pose(10.5, 32, Math.toRadians(0));


    /** Poses for pushing the samples */
    private final Pose pushPose1 = new Pose(36, 40, Math.toRadians(-57));
    private final Pose pushForwardPose1 = new Pose(26, 39, Math.toRadians(-145));
    private final Pose pushPose2 = new Pose(36, 33, Math.toRadians(-57));
    private final Pose pushForwardPose2 = new Pose(26, 29, Math.toRadians(-145));
    private final Pose pushPose3 = new Pose(38, 15, Math.toRadians(-46));
    private final Pose pushForwardPose3 = new Pose(26, 16, Math.toRadians(-160));
    private final Pose moveBackPose = new Pose(20, 20, Math.toRadians(0));

    /** Pose for maneuvering around the submersible */
    private final Pose maneuverPose = new Pose(58, 36.5, Math.toRadians(0));
    /** Maneuver Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the maneuver.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose maneuverControlPose = new Pose(13, 25, Math.toRadians(0));


    private final Pose parkPose = new Pose(12, 30, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, maneuver, park;
    private PathChain moveFirstBlock, moveFirstBlockNet, moveSecondBlock, moveSecondBlockNet, moveThirdBlock, moveThirdBlockNet, grabSpecimen1, grabSpecimen2, grabSpecimen3, grabSpecimen4, scoreSpecimen1, scoreSpecimen2, scoreSpecimen3, scoreSpecimen4;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(score1Pose)));
        //scorePreload.setZeroPowerAccelerationMultiplier(3);
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */





        /* This is our moveBlocks PathChain. We are using multiple paths with a BezierLine, which is a straight line. */
        moveFirstBlock = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(pushPose1)))
                .setPathEndTimeoutConstraint(200)
                .setLinearHeadingInterpolation(score1Pose.getHeading(), pushPose1.getHeading(), 0.5)
                .build();

        moveFirstBlockNet = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushPose1), new Point(pushForwardPose1)))
                .setLinearHeadingInterpolation(pushPose1.getHeading(), pushForwardPose1.getHeading(), 0.5)
                .build();

        moveSecondBlock = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushForwardPose1), new Point(pushPose2)))
                .setPathEndTimeoutConstraint(200)
                .setLinearHeadingInterpolation(pushForwardPose1.getHeading(), pushPose2.getHeading(), 0.5)
                .build();

        moveSecondBlockNet = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushPose2), new Point(pushForwardPose2)))
                .setLinearHeadingInterpolation(pushPose2.getHeading(), pushForwardPose2.getHeading(), 0.5)
                .build();

        moveThirdBlock = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushForwardPose2), new Point(pushPose3)))
                .setPathEndTimeoutConstraint(200)
                .setLinearHeadingInterpolation(pushForwardPose2.getHeading(), pushPose3.getHeading(), 0.5)
                .build();

        moveThirdBlockNet = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushPose3), new Point(pushForwardPose3)))
                .setLinearHeadingInterpolation(pushPose3.getHeading(), pushForwardPose3.getHeading(), 0.5)
                .build();






        /* This is our grabSpecimen1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(grabBackPose)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), grabPose.getHeading(), 0.7)
                .setPathEndTimeoutConstraint(400)
                .setPathEndHeadingConstraint(.001)
                .addPath(new BezierLine(new Point(grabBackPose), new Point(grabPose)))
                .setLinearHeadingInterpolation(grabBackPose.getHeading(), grabPose.getHeading())
                .build();

        scoreSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPose), new Point(score2Pose)))
                .setLinearHeadingInterpolation(grabPose.getHeading(), score2Pose.getHeading(), 0.7)
                .setPathEndTimeoutConstraint(400)
                .setPathEndHeadingConstraint(.001)
                .build();

        grabSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2Pose), new Point(grabBackPose)))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), grabPose.getHeading(), 0.7)
                .setPathEndTimeoutConstraint(400)
                .setPathEndHeadingConstraint(.001)
                .addPath(new BezierLine(new Point(grabBackPose), new Point(grabPose)))
                .setLinearHeadingInterpolation(grabBackPose.getHeading(), grabPose.getHeading())
                .build();

        scoreSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPose), new Point(score3Pose)))
                .setLinearHeadingInterpolation(grabPose.getHeading(), score3Pose.getHeading(), 0.7)
                .setPathEndTimeoutConstraint(400)
                .setPathEndHeadingConstraint(.001)
                .build();

        grabSpecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score3Pose), new Point(grabBackPose)))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), grabPose.getHeading(), 0.7)
                .setPathEndTimeoutConstraint(400)
                .setPathEndHeadingConstraint(.001)
                .addPath(new BezierLine(new Point(grabBackPose), new Point(grabPose)))
                .setLinearHeadingInterpolation(grabBackPose.getHeading(), grabPose.getHeading())
                .build();

        scoreSpecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPose), new Point(score4Pose)))
                .setLinearHeadingInterpolation(grabPose.getHeading(), score4Pose.getHeading(), 0.7)
                .setPathEndTimeoutConstraint(400)
                .setPathEndHeadingConstraint(.001)
                .build();

        grabSpecimen4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score4Pose), new Point(grabBackPose)))
                .setLinearHeadingInterpolation(score4Pose.getHeading(), grabPose.getHeading(), 0.7)
                .setPathEndTimeoutConstraint(400)
                .setPathEndHeadingConstraint(.001)
                .addPath(new BezierLine(new Point(grabBackPose), new Point(grabPose)))
                .setLinearHeadingInterpolation(grabBackPose.getHeading(), grabPose.getHeading())
                .build();
        scoreSpecimen4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPose), new Point(score5Pose)))
                .setLinearHeadingInterpolation(grabPose.getHeading(), score5Pose.getHeading(), 0.7)
                .setPathEndTimeoutConstraint(400)
                .setPathEndHeadingConstraint(.001)
                .build();

        park = new Path(new BezierLine(new Point(score2Pose), new Point(parkPose)));
        park.setTangentHeadingInterpolation();
        //TODO: change back if necessary
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);
                actions.closeClaw();
                actions.clawVertical();
                actions.highChamber();
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                - other robot position: "if(follower.getPose().getX() > (pickup3Pose.getX() - 1) && follower.getPose().getY() > (pickup3Pose.getY() - 1)) {}"

                */

                if(!follower.isBusy()) {
                    /* Score Preload */

                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        actions.highChamberDown();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        actions.openClawLarge();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2.1) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(moveFirstBlock, true);
                        actions.slidesResting();
                        actions.clawEvenMoreVertical();
                        actions.clawRotateServo.setPosition(0.3);
                        setPathState(2);
                    }

                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    actions.extendClaw();
                    actions.clawHover();


                    if(pathTimer.getElapsedTimeSeconds() > 2) {
                        actions.clawDown();

                    }
                    if(pathTimer.getElapsedTimeSeconds() > 2.2) {
                        actions.closeClaw();
                        follower.followPath(moveFirstBlockNet, true);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    actions.openClawLarge();
                    actions.clawHover();
                    actions.clawRotateServo.setPosition(0.35);
                    follower.followPath(moveSecondBlock, true);
                    setPathState(4);

                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                        actions.clawDown();
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 1.7) {
                        actions.closeClaw();
                        follower.followPath(moveSecondBlockNet, true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    actions.openClawLarge();
                    actions.clawHover();
                    actions.retractClaw();
                    actions.clawRotateServo.setPosition(0.4);
                    follower.followPath(moveThirdBlock, true);
                    setPathState(6);

                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 1.7) {
                        actions.clawDown();
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 1.9) {
                        actions.closeClaw();
                        follower.followPath(moveThirdBlockNet, true);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    actions.extendClaw();
                    if(pathTimer.getElapsedTimeSeconds() > 1.5){
                        actions.openClawLarge();
                        actions.clawEvenMoreVertical();
                        actions.retractClaw();
                        actions.clawRotateServo.setPosition(0.5);
                        follower.followPath(grabSpecimen1, true);
                        setPathState(8);
                    }

                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 2.2){

                    }
                    if(pathTimer.getElapsedTimeSeconds() > 2.3){

                    }
                    if(pathTimer.getElapsedTimeSeconds() > 2.4){

                        actions.highChamber();
                        if(actions.slides.getCurrentRightPosition() > 100){
                            follower.followPath(scoreSpecimen1, true);
                            setPathState(9);
                        }
                    }
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    if (pathTimer.getElapsedTimeSeconds() > 3) {
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 4) {
                        actions.slidesResting();
                        follower.followPath(grabSpecimen2, true);
                        setPathState(10);

                    }



                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 2.2){
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 2.4){
                        actions.highChamber();
                        if(actions.slides.getCurrentRightPosition() > 100) {
                            follower.followPath(scoreSpecimen2, true);
                            setPathState(11);
                        }
                    }
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    if (pathTimer.getElapsedTimeSeconds() > 3) {
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 4) {
                        actions.slidesResting();
                        follower.followPath(grabSpecimen3, true);
                        setPathState(12);

                    }



                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 2.2){
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 2.4){
                        actions.highChamberSpecimenClaw();
                        if(actions.slides.getCurrentRightPosition() > 100) {
                            follower.followPath(scoreSpecimen3, true);
                            setPathState(13);
                        }
                    }
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    if (pathTimer.getElapsedTimeSeconds() > 3) {
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 4) {
                        actions.slidesResting();
                        follower.followPath(grabSpecimen4, true);
                        setPathState(14);

                    }



                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 2.2){
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 2.4){
                        actions.highChamberSpecimenClaw();
                        if(actions.slides.getCurrentRightPosition() > 100) {
                            follower.followPath(scoreSpecimen4, true);
                            setPathState(15);
                        }
                    }
                }
                break;
            case 16:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    if (pathTimer.getElapsedTimeSeconds() > 3) {
                        actions.highChamberDownSpecimenClaw();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 4) {
                        actions.slidesResting();
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(park, true);
                        setPathState(-1);
                    }



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

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();



    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        actions = new Subsystem(hardwareMap);

        // Sets the max power to the drive train
        follower.setMaxPower(1);

        // Set the claw to positions for init
        actions.closeClaw();
        actions.retractClaw();
        actions.clawRotateServo.setPosition(0.5);
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
}
