package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.messages.ThreeDeadWheelInputsMessage;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    public static class Params {
        public double rightBackYTicks = -1693.6080197969009; // y position of the first parallel encoder (in tick units)
        public double rightFrontYTicks = 1674.9370157997016; // y position of the second parallel encoder (in tick units)
        public double leftBackXTicks = -2119.024952154118; // x position of the leftBackendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final Encoder rightBack, rightFront, leftBack;

    public final double inPerTick;

    private int lastrightBackPos, lastrightFrontPos, lastleftBackPos;
    private boolean initialized;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        rightBack = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBack"))); //par0
        rightFront = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightFront"))); //par1
        leftBack = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftBack"))); //perp

        // TODO: reverse encoder directions if needed
        //   rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        this.inPerTick = inPerTick;

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
        PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();
        PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();

        FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(rightBackPosVel, rightFrontPosVel, leftBackPosVel));

        if (!initialized) {
            initialized = true;

            lastrightBackPos = rightBackPosVel.position;
            lastrightFrontPos = rightFrontPosVel.position;
            lastleftBackPos = leftBackPosVel.position;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int rightBackPosDelta = rightBackPosVel.position - lastrightBackPos;
        int rightFrontPosDelta = rightFrontPosVel.position - lastrightFrontPos;
        int leftBackPosDelta = leftBackPosVel.position - lastleftBackPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.rightBackYTicks * rightFrontPosDelta - PARAMS.rightFrontYTicks * rightBackPosDelta) / (PARAMS.rightBackYTicks - PARAMS.rightFrontYTicks),
                                (PARAMS.rightBackYTicks * rightFrontPosVel.velocity - PARAMS.rightFrontYTicks * rightBackPosVel.velocity) / (PARAMS.rightBackYTicks - PARAMS.rightFrontYTicks),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                (PARAMS.leftBackXTicks / (PARAMS.rightBackYTicks - PARAMS.rightFrontYTicks) * (rightFrontPosDelta - rightBackPosDelta) + leftBackPosDelta),
                                (PARAMS.leftBackXTicks / (PARAMS.rightBackYTicks - PARAMS.rightFrontYTicks) * (rightFrontPosVel.velocity - rightBackPosVel.velocity) + leftBackPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        (rightBackPosDelta - rightFrontPosDelta) / (PARAMS.rightBackYTicks - PARAMS.rightFrontYTicks),
                        (rightBackPosVel.velocity - rightFrontPosVel.velocity) / (PARAMS.rightBackYTicks - PARAMS.rightFrontYTicks),
                })
        );

        lastrightBackPos = rightBackPosVel.position;
        lastrightFrontPos = rightFrontPosVel.position;
        lastleftBackPos = leftBackPosVel.position;

        return twist;
    }
}
