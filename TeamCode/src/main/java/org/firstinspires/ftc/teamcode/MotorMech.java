package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorMech {
    private static DcMotorEx right, left;

    public MotorMech(HardwareMap hardwareMap){
        right = hardwareMap.get(DcMotorEx.class, "right");
        left = hardwareMap.get(DcMotorEx.class, "left");
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setDirection(DcMotorEx.Direction.REVERSE);
        left.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public static class MotorUp implements Action{
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                right.setPower(0.8);
                left.setPower(0.8);
                initialized = true;
            }

            // checks lift's current position
            packet.put("rightPos", right.getCurrentPosition());
            packet.put("leftPos", left.getCurrentPosition());
            if (right.getCurrentPosition() < 3000.0) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                right.setPower(0);
                return false;
            }
            // overall, the action powers the lift until it surpasses
            // 3000 encoder ticks, then powers it off
        }
    }

    public static Action motorUp(){
        return new MotorUp();
    }

    public class LiftDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                right.setPower(-0.8);
                left.setPower(-0.8);
                initialized = true;
            }

            packet.put("rightPos", right.getCurrentPosition());
            packet.put("leftPos", left.getCurrentPosition());
            packet.put("liftPos", right.getCurrentPosition());
            if (right.getCurrentPosition() > 100.0) {
                return true;
            } else {
                right.setPower(0);
                left.setPower(0);
                return false;
            }
        }
    }

    public Action liftDown() {
        return new LiftDown();
    }


}