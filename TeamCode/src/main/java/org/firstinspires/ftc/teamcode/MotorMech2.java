package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MotorMech2 {
    public DcMotorEx left, right, hang;
    public int targetPosition;

    public double power = 0;

    public MotorMech2(@NonNull HardwareMap hardwareMap, double power, boolean craneByPower){
        left = hardwareMap.get(DcMotorEx.class, "left");
        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        right = hardwareMap.get(DcMotorEx.class, "right");
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        right.setDirection(DcMotorEx.Direction.REVERSE);
        left.setDirection(DcMotorEx.Direction.FORWARD);
        resetEncoders();

        hang = hardwareMap.get(DcMotorEx.class, "hang");
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setTargetPosition(0);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setDirection(DcMotorEx.Direction.REVERSE);


        targetPosition = 0;

        setPower(power);
        this.power = power;
        setTargetPosition(0);
        setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void setTargetPosition(int target){
        setPower(0.8);
        left.setTargetPosition(target);
        right.setTargetPosition(target);
        targetPosition = target;
    }

    public void setTargetPosition(int target, double power){
        setPower(power);
        left.setTargetPosition(target);
        right.setTargetPosition(target);
        targetPosition = target;
    }

    public void resetEncoders(){
        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        setTargetPosition(0);
        setPower(power);

        left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void craneMaintenance(){
        if(left.getCurrentPosition() < 50 && left.getCurrent(CurrentUnit.AMPS) > 0.5 && left.getTargetPosition() == 0){
            left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            left.setTargetPosition(0);
            left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            left.setPower(0);
        }
        if(right.getCurrentPosition() < 50 && right.getCurrent(CurrentUnit.AMPS)  > 0.5 && right.getTargetPosition() == 0){
            right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            right.setTargetPosition(0);
            right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            right.setPower(0);
        }
    }

    public void move(double movement, boolean byPower){
        if(movement > 0 && byPower){
            setTargetPosition(3600, movement);
        }else if(movement < 0 && byPower){
            setTargetPosition(0, -movement);
        }else if(byPower){
            setPower(0);
        }else if(movement > 3600){
            setTargetPosition(3600);
        }else if(movement < 0){
            setTargetPosition(0);
        }else{
            setTargetPosition((int)movement);
        }
    }
    public void setPower(double power){
        left.setPower(power);
        right.setPower(power);
    }

    public int getCurrentLeftPosition() { return left.getCurrentPosition(); }

    public int getCurrentRightPosition() { return right.getCurrentPosition(); }

    public boolean offCheck(){ return getCurrentLeftPosition() < 0 || getCurrentRightPosition() < 0; }


    public void setMode(DcMotorEx.RunMode mode){
        left.setMode(mode);
        right.setMode(mode);
    }


    public double getAmpsLeft(){return left.getCurrent(CurrentUnit.AMPS);}
    public double getAmpsRight(){return right.getCurrent(CurrentUnit.AMPS);}

    public void setHang(){
        movevertically(hang, 1800, 1);
    }


    public void movevertically(DcMotorEx lipsey, int position, double power) {
        untoPosition(lipsey);
        runtoPosition(lipsey);
        lipsey.setTargetPosition(position);
        lipsey.setPower(power);
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