package org.firstinspires.ftc.teamcode.PPTuning;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;

public class FConstants {
    static {
        // Select our localizer
        FollowerConstants.localizers = Localizers.TWO_WHEEL;
        // We can change the value of any variable/constant of FollowerConstants.
        FollowerConstants.mass = 17.2;
        //TODO: TUNE THIS SHIT
        //FollowerConstants.xMovement = ForwardVeloTuner
        //FollowerConstants.yMovement = StrafeVeloTuner
        //FollowerConstants.forwardZeroPowerAcceleration = ForwardZeroPowerAcceleration
    }
}
