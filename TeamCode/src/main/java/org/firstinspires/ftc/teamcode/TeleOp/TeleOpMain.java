package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    private ServoImplEx backR, backL, frontR, frontL, extendR, extendL;

    private int errorBound = 60;
    int height;

    public enum state {
        PRESET,
        CUSTOM
    };

    state drawerState = state.PRESET;

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
        extendL = hardwareMap.get(ServoImplEx.class, "extendR");
        extendR = hardwareMap.get(ServoImplEx.class, "extendL");

        telemetry.update();

        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        right.setDirection(DcMotorEx.Direction.REVERSE);
        left.setDirection(DcMotorEx.Direction.FORWARD);

        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        reset();

        right.setCurrentAlert(1, CurrentUnit.AMPS);

        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setTargetPosition(0);
        hang.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        servoTimer.reset();
        telemetry.update();
        drawerState = state.PRESET;

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
//
//                        }

                        break;
                    case PRESET:
                        if (gamepad2.triangle) {
                            setDrawerHeight(3600); // top basket good
                            //waitforDrawers(right, left);
                            //extendR.setPosition(0.70);
                            //extendL.setPosition(0.30);
                        } else if (gamepad2.circle) {
                            setDrawerHeight(1800);
                        } else if (gamepad2.cross) {
                            setDrawerHeight(1500);
                        }

                        if(gamepad2.square){
                            drawerTimer.reset();
                            setDrawerHeight(0);
                            if(drawersDone(right, left) && drawerTimer.seconds() > 3){
                                settle_slides();
                            }
                        }

                    default:
                        drawerState = state.PRESET;
                }

                //OTHER GAMEPAD2 CONTROLS

                //servos
                if(gamepad2.dpad_up){
                    extendR.setPosition(0.70);
                    extendL.setPosition(0.30);
                }

                if(gamepad2.dpad_right){
                    extendR.setPosition(0.85);
                    extendL.setPosition(0.15);
                }

                if(gamepad2.dpad_down){
                    extendR.setPosition(1);
                    extendL.setPosition(0);
                }

                //hanger
                if(gamepad2.right_stick_y > 0){
                    //hang.setTargetPosition(4500);
                    hang.setPower(gamepad2.right_stick_y);
                }
                else if(gamepad2.right_stick_y < 0){
                    //hang.setTargetPosition(0);
                    hang.setPower(gamepad2.right_stick_y);
                }
                else{
                    hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    hang.setPower(0);
                }

                //GAMEPAD1 CONTROLS

                //drivetrain
                rightFront.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x)) + (gamepad1.right_stick_x));
                leftFront.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x)) + ((gamepad1.right_stick_x)));
                rightBack.setPower(((gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (gamepad1.right_stick_x));
                leftBack.setPower(((-gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (gamepad1.right_stick_x));

                //servos
                if(gamepad1.left_bumper){
                    frontR.setPosition(1);
                    frontL.setPosition(0);
                }

                if(gamepad1.right_bumper){
                    frontR.setPosition(0.65);
                    frontL.setPosition(0.35);
                }

                if(gamepad1.left_trigger > 0){
                    backR.setPosition(1);
                    backL.setPosition(0);
                }

                if(gamepad1.right_trigger > 0){
                    backR.setPosition(0.75);
                    backL.setPosition(0.30);
                }

                //drawer heights
                if(gamepad1.cross){
                    setDrawerHeight(550);
                }

                if(gamepad1.circle){
                    setDrawerHeight(right.getCurrentPosition() + 100);
                }

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
        lipsey.setTargetPosition(position);
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

    public void settle_slides(){
        if(left.getCurrentPosition() < 10 && left.getCurrentAlert(CurrentUnit.AMPS) > 0.5 && left.getTargetPosition() == 0){
            left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            left.setTargetPosition(0);
            left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            left.setPower(0);

        }
        if(right.getCurrentPosition() < 10 && right.getCurrentAlert(CurrentUnit.AMPS) > 0.5 && left.getTargetPosition() == 0){
            right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            left.setTargetPosition(0);
            right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            right.setPower(0);
        }
    }

}