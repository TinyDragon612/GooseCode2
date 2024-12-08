package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name= "SlideTester", group="Linear Opmode")
public class SlideTester extends LinearOpMode {

    PIDFController vert = new PIDFController(0.05, 0.001, 0.01, 0.005);

    @Override
    public void runOpMode(){
        DcMotorEx right, left;
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
                right.setPower(gamepad1.left_trigger);
                left.setPower(gamepad1.left_trigger);
                usePIDF = false;
            }
            else if (gamepad1.right_trigger > 0){
                right.setPower(-gamepad1.right_trigger);
                left.setPower(-gamepad1.right_trigger);
                usePIDF = false;
            }
            else if(usePIDF){
                right.setPower(0);
                left.setPower(0);

                if(gamepad1.cross){
                    right.setPower(vert.calculate(right.getCurrentPosition(), TargetPosition));
                    left.setPower(vert.calculate(right.getCurrentPosition(), TargetPosition));
                }

            }

        }

    }

}