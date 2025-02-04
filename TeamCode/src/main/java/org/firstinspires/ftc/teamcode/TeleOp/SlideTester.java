package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//@Disabled
@TeleOp(name= "SlideTester", group="Linear Opmode")
public class SlideTester extends LinearOpMode {

    PIDFController vert = new PIDFController(0.05, 0.001, 0.01, 0.005);

    DcMotorEx right, left;
    ElapsedTime drawerTimer = new ElapsedTime();

    @Override
    public void runOpMode(){
        waitForStart();

        int TargetPosition = 2000;

        boolean usePIDF = false;

        right = hardwareMap.get(DcMotorEx.class, "right");
        left = hardwareMap.get(DcMotorEx.class, "left");

        right.setDirection(DcMotorEx.Direction.REVERSE);
        left.setDirection(DcMotorEx.Direction.FORWARD);

        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Gamepad lastGamepad1 = new Gamepad();
        //Gamepad lastGamepad2 = new Gamepad();

        while (opModeIsActive()){

            telemetry.addData("right: ", right.getCurrentPosition());
            telemetry.addData("left: ", left.getCurrentPosition());
            telemetry.addData("right run mode: ", right.getMode());
            telemetry.addData("Powerrrr:", right.isOverCurrent());
            telemetry.update();

            if(gamepad1.a && !lastGamepad1.a){
                usePIDF = true;
            }

            if (gamepad1.left_trigger > 0){
                right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                right.setPower(gamepad1.left_trigger);
                left.setPower(gamepad1.left_trigger);
                usePIDF = false;
            }
            else if (gamepad1.right_trigger > 0){
                right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                right.setPower(-gamepad1.right_trigger);
                left.setPower(-gamepad1.right_trigger);
                usePIDF = false;
            }
            else if(usePIDF) {
                right.setPower(0);
                left.setPower(0);

                if(gamepad2.cross){
                    right.setPower(vert.calculate(right.getCurrentPosition(), 1000));
                }
                if(gamepad2.circle){
                    left.setPower(vert.calculate(right.getCurrentPosition(), 2500));
                }

//                if (gamepad2.triangle) {
//                    setDrawerHeight(3600); // top basket good
//                } else if (gamepad2.circle) {
//                    setDrawerHeight(1800);
//                } else if (gamepad2.cross) {
//                    setDrawerHeight(1500);
//                }
//                else if(gamepad2.square){
//                    drawerTimer.reset();
//                    setDrawerHeight(0);
//                    if(drawersDone(right, left) && drawerTimer.seconds() > 3){
//                        settle_slides();
//                    }
//                }
            }

        }

    }

    public void setDrawerHeight(int h){
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        movevertically(right, h, 1);
        movevertically(left, h, 1);
    }

    public void movevertically(DcMotorEx lipsey, int position, double power) {
        untoPosition(lipsey);
        runtoPosition(lipsey);
        lipsey.setTargetPosition(position);
        right.setPower(vert.calculate(right.getCurrentPosition(), position));
        left.setPower(vert.calculate(right.getCurrentPosition(), position));
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

    public boolean drawersDone(DcMotor george, DcMotor BobbyLocks) {
        return ((george.getCurrentPosition() > george.getTargetPosition() - 60 && george.getCurrentPosition() < george.getTargetPosition() + 60) &&
                (BobbyLocks.getCurrentPosition() > BobbyLocks.getTargetPosition() - 60 && BobbyLocks.getCurrentPosition() < BobbyLocks.getTargetPosition() + 60));
    }

}