package org.firstinspires.ftc.teamcode.PPTuning;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        // Select our localizer
        FollowerConstants.localizers = Localizers.TWO_WHEEL;
        // We can change the value of any variable/constant of FollowerConstants.
        FollowerConstants.mass = 17.2;
        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftBack";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightBack";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.xMovement = 66.36726243394294;
        FollowerConstants.yMovement = 50.969302940009094;
        FollowerConstants.forwardZeroPowerAcceleration = -26.00670147829049;
        FollowerConstants.lateralZeroPowerAcceleration = -61.810000143371646;
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.5,0.002,0.05,0);
        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0.01,0.1,0);
        FollowerConstants.zeroPowerAccelerationMultiplier = 3;
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.013,0,0.00005,0.6,0);
        FollowerConstants.centripetalScaling = 0.0002;
    }
}
