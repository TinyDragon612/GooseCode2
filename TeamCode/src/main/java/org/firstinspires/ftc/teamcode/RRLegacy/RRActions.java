package org.firstinspires.ftc.teamcode.RRLegacy;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class RRActions {
    private ServoImplEx backR;
    private ServoImplEx backL;
    private ServoImplEx frontR;
    private ServoImplEx frontL;
    private ServoImplEx extendR;
    private ServoImplEx extendL;
    public MotorMech2 slides;
    public DcMotorEx hang;


    private double cranePower = 0.1;


    public RRActions(HardwareMap hardwareMap){
        slides = new MotorMech2(hardwareMap, cranePower, false);
        frontR = hardwareMap.get(ServoImplEx.class, "frontR");
        frontL = hardwareMap.get(ServoImplEx.class, "frontL");
        backR = hardwareMap.get(ServoImplEx.class, "backR");
        backL = hardwareMap.get(ServoImplEx.class, "backL");
        extendR = hardwareMap.get(ServoImplEx.class, "extendR");
        extendL = hardwareMap.get(ServoImplEx.class, "extendL");
    }


    public class wallBump implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slides.setTargetPosition(600);
            return false;
        }
    }
    public Action wallBump() {return new RRActions.wallBump();}

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

    public Action highChamber() {return new RRActions.highChamber();}

    public class scoreChamber implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            slides.setTargetPosition(1500);
            return false;
        }
    }

    public Action scoreChamber() {return new RRActions.scoreChamber();}


    public class slideHang implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            slides.setTargetPosition(900);
            return false;
        }
    }

    public Action slideHang() {return new RRActions.slideHang();}


    public class setHang implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slides.setHang();
            return false;
        }
    }

    public Action setHang() {return new RRActions.setHang();}

    public class wall implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slides.setTargetPosition(550);
            return false;
        }
    }

    public Action wall() {return new RRActions.wall();}



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

    public class openBack implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            backR.setPosition(1);
            backL.setPosition(0);
            return false;
        }
    }

    public Action openBack() {
        return new RRActions.openBack();
    }

    public class closeBack implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            backR.setPosition(0.75);
            backL.setPosition(0.30);
            return false;
        }

    }

    public Action closeBack() {
        return new RRActions.closeBack();
    }

    public class extendOut implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendR.setPosition(0.30);
            extendL.setPosition(0.70);
            return false;
        }

    }

    public Action extendOut() {
        return new RRActions.extendOut();
    }

    public class extendIn implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendR.setPosition(0);
            extendL.setPosition(1);
            return false;
        }
    }

    public Action extendIn() {
        return new RRActions.extendIn();
    }

    public class frontOpen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            frontR.setPosition(1);
            frontL.setPosition(0);
            return false;
        }

    }

    public Action frontOpen() {
        return new RRActions.frontOpen();
    }

    //new

    public class frontClose implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            frontR.setPosition(0.65);
            frontL.setPosition(0.40);
            return false;
        }

    }

    public Action frontClose() {
        return new RRActions.frontClose();
    }

}
