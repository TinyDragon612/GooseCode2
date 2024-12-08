package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name= "TeleOpMain", group="Linear Opmode")
//@Disabled
public class TeleOpMain extends LinearOpMode {

    // Declare OpMode members.
    private DcMotorEx.ZeroPowerBehavior brake = DcMotorEx.ZeroPowerBehavior.BRAKE;
    private DcMotorEx.ZeroPowerBehavior floatt = DcMotorEx.ZeroPowerBehavior.FLOAT;

    private DcMotorEx leftFront, leftBack, rightBack, rightFront; //wheels

    private DcMotorEx right, left, hang; //slides

    private ServoImplEx backR, backL, frontR, frontL, extend1, extend2;

    private int errorBound = 60;
    int height;

    public enum state {
        DRAWER_START,
        DRAWER_FLIP_IN,
        DRAWER_FLIP_OUT,
        DRAWER_RETRACT,
        DRAWER_SETTLE,
        CUSTOM

    };

    state drawerState = state.DRAWER_START;

    ElapsedTime drawerTimer = new ElapsedTime();
    ElapsedTime servoTimer = new ElapsedTime();

    ElapsedTime slideTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drawerTimer.reset();

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        right = hardwareMap.get(DcMotorEx.class, "right");
        left = hardwareMap.get(DcMotorEx.class, "left");
        hang = hardwareMap.get(DcMotorEx.class, "hang");
        backR = hardwareMap.get(ServoImplEx.class, "backR");
        backL = hardwareMap.get(ServoImplEx.class, "backL");
        frontR = hardwareMap.get(ServoImplEx.class, "frontR");
        frontL = hardwareMap.get(ServoImplEx.class, "frontL");
//        extend1 = hardwareMap.get(ServoImplEx.class, "extend1");
//        extend2 = hardwareMap.get(ServoImplEx.class, "extend2");

        telemetry.update();

        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        right.setDirection(DcMotorEx.Direction.REVERSE);
        left.setDirection(DcMotorEx.Direction.FORWARD);

        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right.setTargetPosition(0);
        left.setTargetPosition(0);

        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        right.setCurrentAlert(1, CurrentUnit.AMPS);

        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoTimer.reset();
        telemetry.update();
        drawerState = state.DRAWER_START;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry.addData("Status", "Running");
                telemetry.addData("right: ", right.getCurrentPosition());
                telemetry.addData("left: ", left.getCurrentPosition());
                telemetry.addData("hang: ", hang.getCurrentPosition());
                telemetry.update();

                switch (drawerState) {
                    case CUSTOM:
//                        right.setZeroPowerBehavior(brake);
//                        left.setZeroPowerBehavior(brake);
//
//                        //left.setPower(-gamepad2.right_stick_y);
//                        //right.setPower(-gamepad2.right_stick_y);
//
//                        x = -gamepad2.right_stick_y;
//
//                        if(x <= 1 && x > 0){
//                            movevertically(slide1, 9000, x * 10);
//                        }
//                        else if(x >= -1 && x < 0){
//                            movevertically(slide1, 0, x * 10);
//                        }
//                        else{
//                            right.setTargetPositionTolerance(slide1.getCurrentPosition());
//                        }
//
//                        if(gamepad2.left_bumper) {
//                            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            drawerState = state.DRAWER_FLIP_IN;
//                        }

                        break;
                    case DRAWER_START:

                        break;
                    case DRAWER_FLIP_OUT:

//                        if (drawersDone(slide1, slide1)) {
//
//                            swoosh1.setPosition(0);
//                            swoosh2.setPosition(.2);
//                            flop1.setPosition(0.87);
//                            flop2.setPosition(0.13);
//
//                            drawerState = state.DRAWER_FLIP_IN;
//                        }
                        break;

                    case DRAWER_FLIP_IN:

                        if(gamepad2.right_bumper){
                            right.setZeroPowerBehavior(brake);
                            left.setZeroPowerBehavior(brake);
                            drawerState = state.CUSTOM;
                            drawerTimer.reset();
                            drawerState = state.DRAWER_RETRACT;
                        }

                        break;
                    case DRAWER_RETRACT:
//                        if (drawerTimer.seconds() >= 3) {
//                            bringDrawersDown();
//                            drawerTimer.reset();
//                            drawerState = state.DRAWER_SETTLE;
//                        }
                        break;
                    case DRAWER_SETTLE:
//                        if (magnetic.isPressed() || magnetic2.isPressed()) {
//                            pinch1.setPosition(0.35);
//                            pinch2.setPosition(0.9);
//                            untoPosition(slide1);
//                            //untoPosition(slide2);
//                            reset();
//                            drawerState = state.DRAWER_START;
//                        }
                        break;
                    default:
                        drawerState = state.DRAWER_START;
                }

                //OTHER GAMEPAD2 CONTROLS
                if(gamepad1.right_bumper){
                    backR.setPosition(1);
                    backL.setPosition(0);
                }

                if(gamepad1.left_bumper){
                    backR.setPosition(0.75);
                    backL.setPosition(0.30);
                }

                if(gamepad1.dpad_down){
                    setDrawerHeight(-height);
                }

                if (gamepad1.y) {
                    setDrawerHeight(3600); // top basket good
                    drawerState = state.DRAWER_FLIP_OUT;
                } else if (gamepad1.b) {
                    setDrawerHeight(1800);
                    drawerState = state.DRAWER_FLIP_OUT;
                } else if (gamepad1.a) {
                    setDrawerHeight(1500);
                    drawerState = state.DRAWER_FLIP_OUT;
                }

                //GAMEPAD1 CONTROLS


                if(gamepad1.left_trigger > 0){
                    hang.setPower(-gamepad1.left_trigger);
                }
                else if(gamepad1.right_trigger > 0){
                    hang.setPower(gamepad1.right_trigger);
                }
                else{
                    hang.setPower(0);
                }


                rightFront.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x)) + (gamepad1.right_stick_x));
                leftFront.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x)) + ((gamepad1.right_stick_x)));
                rightBack.setPower(((gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (gamepad1.right_stick_x));
                leftBack.setPower(((-gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (gamepad1.right_stick_x));

            }

        }
    }

    public void reset(){
        right.setPower(0);
        left.setPower(0);

        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right.setTargetPosition(0);
        left.setTargetPosition(0);

        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setDrawerHeight(int h){
        height = h;

        movevertically(right, h, 1);
        movevertically(left, h, 1);
    }

    public void waitforDrawer(DcMotor george) {
        while(!(george.getCurrentPosition() > george.getTargetPosition() - errorBound && george.getCurrentPosition() < george.getTargetPosition() + errorBound));
    }

    public boolean waitforDrawers(DcMotor george, DcMotor BobbyLocks) {
        return ((george.getCurrentPosition() > george.getTargetPosition() - errorBound && george.getCurrentPosition() < george.getTargetPosition() + errorBound) &&
                (BobbyLocks.getCurrentPosition() > BobbyLocks.getTargetPosition() - errorBound && BobbyLocks.getCurrentPosition() < BobbyLocks.getTargetPosition() + errorBound));
    }

    public boolean drawersDone(DcMotor george, DcMotor BobbyLocks) {
        return ((george.getCurrentPosition() > george.getTargetPosition() - errorBound && george.getCurrentPosition() < george.getTargetPosition() + errorBound) &&
                (BobbyLocks.getCurrentPosition() > BobbyLocks.getTargetPosition() - errorBound && BobbyLocks.getCurrentPosition() < BobbyLocks.getTargetPosition() + errorBound));
    }

    public void movevertically(DcMotorEx lipsey, int position, double power) {
        untoPosition(lipsey);
        runtoPosition(lipsey);
        lipsey.setTargetPosition(lipsey.getCurrentPosition() + position);
        lipsey.setPower(power);
    }

    public void nostall(DcMotorEx Harry) {
        Harry.setZeroPowerBehavior(floatt);
        Harry.setPower(0);
    }

    public void stall(DcMotorEx DcMotar) {
        DcMotar.setZeroPowerBehavior(brake);
        DcMotar.setPower(0);
    }

    public void runtoPosition(DcMotorEx John) {
        John.setTargetPosition(0);
        John.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        John.setPower(0);
    }
    public void untoPosition(DcMotorEx Neil) {
        Neil.setPower(0);
        Neil.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

}