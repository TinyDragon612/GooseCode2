package org.firstinspires.ftc.teamcode.PPTuning;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.TwoWheelConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        //TODO: FIX THIS SHIT
        TwoWheelConstants.forwardTicksToInches = 0.0029348203; //FIX
        TwoWheelConstants.strafeTicksToInches = 0.0013051277405697619; //FIX
        TwoWheelConstants.forwardY = 1; //FIX
        TwoWheelConstants.strafeX = -2.5; //FIX
        TwoWheelConstants.forwardEncoder_HardwareMapName = "leftFront";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "rightFront";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }
}
