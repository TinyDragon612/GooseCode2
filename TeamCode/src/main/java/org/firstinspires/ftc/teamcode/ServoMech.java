package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ServoMech {
    private static ServoImplEx backR;
    private static ServoImplEx backL;
    private static ServoImplEx frontR;
    private static ServoImplEx frontL;
    private static ServoImplEx extendR;
    private static ServoImplEx extendL;

    public ServoMech(HardwareMap hardwareMap){
        frontR = hardwareMap.get(ServoImplEx.class, "frontR");
        frontL = hardwareMap.get(ServoImplEx.class, "frontL");
        backR = hardwareMap.get(ServoImplEx.class, "backR");
        backL = hardwareMap.get(ServoImplEx.class, "backL");
        extendR = hardwareMap.get(ServoImplEx.class, "extendR");
        extendL = hardwareMap.get(ServoImplEx.class, "extendL");
    }

    public static class CloseFront implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            frontR.setPosition(0.55);
            frontL.setPosition(1);
            return false;
        }
    }
    public static Action closeFront() {
        return new CloseFront();
    }

    public class OpenClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            frontR.setPosition(0.55);
            frontL.setPosition(1);
            return false;
        }
    }
    public Action openFront() {
        return new OpenClaw();
    }

}
