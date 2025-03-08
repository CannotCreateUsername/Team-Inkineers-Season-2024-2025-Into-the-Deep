package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = .003;
        ThreeWheelConstants.strafeTicksToInches = .003;
        ThreeWheelConstants.turnTicksToInches = 0.0029;
        // According to CAD left and right should total 12.283 or 12.284 inches
        // Currently adds to 12.21658
        ThreeWheelConstants.leftY = 6.5438;     //  6.142;
        ThreeWheelConstants.rightY = -5.6727;   // -6.142;
        ThreeWheelConstants.strafeX = -4.2828;  // -3.791;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftFront";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "rightBack";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "leftBack";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




