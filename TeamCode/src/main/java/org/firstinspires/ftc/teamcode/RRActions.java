package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class RRActions {
    private ServoImplEx backR;
    private ServoImplEx backL;
    private ServoImplEx frontR;
    private ServoImplEx frontL;
    private ServoImplEx extendR;
    private ServoImplEx extendL;
    public MotorMech2 slides;


    private double cranePower = 0.1;

    private double extendAmount;


    public RRActions(HardwareMap hardwareMap){
        slides = new MotorMech2(hardwareMap, cranePower, false);
        frontR = hardwareMap.get(ServoImplEx.class, "frontR");
        frontL = hardwareMap.get(ServoImplEx.class, "frontL");
        backR = hardwareMap.get(ServoImplEx.class, "backR");
        backL = hardwareMap.get(ServoImplEx.class, "backL");
        extendR = hardwareMap.get(ServoImplEx.class, "extendR");
        extendL = hardwareMap.get(ServoImplEx.class, "extendL");
    }

    public class openClaw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //clawServo.setPosition(values.clawOpen);
            return false;
        }


    }

    public Action openClaw() {
        return new RRActions.openClaw();
    }



    public class closeClaw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //clawServo.setPosition(values.clawClsoed);
            return false;
        }


    }

    public Action closeClaw() {
        return new RRActions.closeClaw();
    }



    public class extendClaw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            //extenderRight.setPosition(1-values.clawExtend);
            //extenderLeft.setPosition(values.clawExtend);
            return false;
        }


    }

    public Action extendClaw() {
        return new RRActions.extendClaw();
    }


    public class moveClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            //extenderRight.setPosition(1-extendAmount);
            //extenderLeft.setPosition(extendAmount);
            return false;
        }


    }

    public Action moveClaw(double length) {
        extendAmount = length;
        return new RRActions.moveClaw();
    }



    public class retractClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            //extenderRight.setPosition(1-values.clawRetract);
            //extenderLeft.setPosition(values.clawRetract);
            return false;
        }


    }

    public Action retractClaw() {
        return new RRActions.retractClaw();
    }

    public class clawUp implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            //clawTurnLeft.setPosition(0.20);
            //clawTurnRight.setPosition(0.8);
            return false;
        }


    }

    public Action clawUp() {
        return new RRActions.clawUp();
    }

    public class clawDown implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            //clawTurnLeft.setPosition(0.42);
            //clawTurnRight.setPosition(0.58);
            return false;
        }


    }

    public Action clawDown() {
        return new RRActions.clawDown();
    }



    public class clawVertical implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            //clawTurnLeft.setPosition(0.1);
            //clawTurnRight.setPosition(0.9);
            return false;
        }


    }

    public Action clawVertical() {
        return new RRActions.clawVertical();
    }




    public class highBasket implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            slides.setTargetPosition(3600);
            return false;
        }


    }

    public Action highBasket() {
        return new RRActions.highBasket();
    }



    public class highChamber implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            slides.setTargetPosition(1800);
            return false;
        }


    }

    public Action highChamber() {
        return new RRActions.highChamber();
    }



    public class slidesDown implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            slides.setTargetPosition(0);
            return false;
        }


    }

    public Action slidesDown() {
        return new RRActions.slidesDown();
    }



    public class specimenOpen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            //specimenLeft.setPosition(0.65);
            //specimenRight.setPosition(0.7);
            return false;
        }


    }

    public Action specimenOpen() {
        return new RRActions.specimenOpen();
    }



    public class specimenClose implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            //specimenLeft.setPosition(0.5);
            //specimenRight.setPosition(0.8);
            return false;
        }


    }

    public Action specimenClose() {
        return new RRActions.specimenClose();
    }






}
