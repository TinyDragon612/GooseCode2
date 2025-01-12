package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Disabled
@TeleOp(name= "BabyMode", group="Linear Opmode")
//@Disabled
public class BabyMode extends LinearOpMode {

    // Declare OpMode members.
    private DcMotorEx.ZeroPowerBehavior brake = DcMotorEx.ZeroPowerBehavior.BRAKE;
    private DcMotorEx.ZeroPowerBehavior floatt = DcMotorEx.ZeroPowerBehavior.FLOAT;

    private DcMotorEx leftFront, leftBack, rightBack, rightFront; //wheels

    private DcMotorEx right, left, hang; //slides

    private ServoImplEx backR, backL, frontR, frontL, extendR, extendL;

    private int errorBound = 60;
    int height;
    boolean holding = false;
    //you can delete these two if you want, they're used for craig's button thingy.
    boolean clawState = false;
    boolean motorState = true;
    boolean pressed = false;
    int notPressed = 0;

    public enum state {
        PRESET,
        CUSTOM
    }

    ;

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

        right.setCurrentAlert(1, CurrentUnit.AMPS);

        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setTargetPosition(0);
        //hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setDirection(DcMotorEx.Direction.REVERSE);

        servoTimer.reset();
        telemetry.update();
        drawerState = state.PRESET;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                rightFront.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x)* 0.25) + (gamepad1.right_stick_x) * 0.25);
                leftFront.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x)* 0.25) + (gamepad1.right_stick_x) * 0.25);
                rightBack.setPower(((gamepad1.left_stick_y + -gamepad1.left_stick_x)* 0.25) + (gamepad1.right_stick_x)* 0.25 );
                leftBack.setPower(((-gamepad1.left_stick_y + -gamepad1.left_stick_x)* 0.25) + (gamepad1.right_stick_x)* 0.25);

                if (gamepad1.left_bumper) {
                    frontR.setPosition(1);
                    frontL.setPosition(0);
                    clawState = false;
                }

                if (gamepad1.right_bumper) {
                    if (clawState) {
                        frontR.setPosition(0.8);
                        frontL.setPosition(0.20);

                    } else {
                        frontR.setPosition(0.60);
                        frontL.setPosition(0.40);
                    }
                    if (notPressed > 20) {
                        clawState = !clawState;
                    }
                    notPressed = 0;

                } else {
                    if (notPressed > 39) {
                        notPressed = 40;
                    } else {
                        notPressed += 1;
                    }
                }

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
            }
        }
    }
}