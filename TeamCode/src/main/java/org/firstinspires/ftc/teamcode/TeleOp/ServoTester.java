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

@TeleOp(name= "ServoTester", group="Linear Opmode")
//@Disabled
public class ServoTester extends LinearOpMode {


    private ServoImplEx backR, backL, extendR, extendL, frontR, frontL;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        frontR = hardwareMap.get(ServoImplEx.class, "frontR");
        frontL = hardwareMap.get(ServoImplEx.class, "frontL");
        backR = hardwareMap.get(ServoImplEx.class, "backR");
        backL = hardwareMap.get(ServoImplEx.class, "backL");
        extendR = hardwareMap.get(ServoImplEx.class, "extendR");
        extendL = hardwareMap.get(ServoImplEx.class, "extendL");

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry.addData("Status", "Running");
                telemetry.addData("right: ", backR.getPosition());
                telemetry.addData("left: ", backL.getPosition());
                telemetry.update();

//                if(gamepad1.dpad_up){
//                    extendL.setPosition(1);
//                }
//
//                if(gamepad1.dpad_down){
//                    extendL.setPosition(0); // out position
//                }
//
//                if(gamepad1.triangle){
//                    extendR.setPosition(1); // out position
//                }
//
//                if(gamepad1.cross){
//                    extendR.setPosition(0);
//                }
//
//                if(gamepad1.right_bumper){
//                    extendR.setPosition(1);
//                    extendL.setPosition(0);
//
//                }
                

//                BACK CLAWS
//                if(gamepad1.right_bumper){
//                    backR.setPosition(1);
//                    backL.setPosition(0);
//
//                }
//                
//                if(gamepad1.left_bumper){
//                    backR.setPosition(0.75);
//                    backL.setPosition(0.30);
//                }


                //FRONT CLAWS
                if(gamepad1.right_bumper){
                    frontR.setPosition(1);
                    frontL.setPosition(0);

                }

                if(gamepad1.left_bumper){
                    frontR.setPosition(0);
                    frontL.setPosition(1);
                }

                if(gamepad1.cross){
                    frontR.setPosition(0.70);
                    frontL.setPosition(0.30);
                }

                if(gamepad1.circle){
                    frontR.setPosition(0.8);
                    frontL.setPosition(0.);
                }
                

            }

        }
    }
}