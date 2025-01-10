package org.firstinspires.ftc.teamcode.PPTuning;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.TwoWheelConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.002937;
        TwoWheelConstants.strafeTicksToInches = 0.002977;
        TwoWheelConstants.forwardY = -4.8125;
        TwoWheelConstants.strafeX = -6.34375;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "rightFront";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "leftBack";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }
}
