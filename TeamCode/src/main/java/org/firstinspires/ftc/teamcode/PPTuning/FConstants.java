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

        FollowerConstants.xMovement = 57.3197;
        FollowerConstants.yMovement = 41.7313;
        FollowerConstants.forwardZeroPowerAcceleration = -35.2608;
        FollowerConstants.lateralZeroPowerAcceleration = -69.787;
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.5,0,0.05,0);
        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.1,0);
        FollowerConstants.zeroPowerAccelerationMultiplier = 10;
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01,0,0.0004,2,0); // 0.007, 0, 0.00005
        FollowerConstants.centripetalScaling = 0.0002;
    }
}
