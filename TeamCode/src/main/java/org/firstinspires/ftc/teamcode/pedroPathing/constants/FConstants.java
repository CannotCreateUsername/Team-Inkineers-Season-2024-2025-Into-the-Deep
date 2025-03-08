package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.THREE_WHEEL;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftBack";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightBack";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 15.8; // In kilograms

//        FollowerConstants.xMovement = 90.5; // changed 3/4/25 // 78.80
//        FollowerConstants.yMovement = 66.0; // changed 3/4/25 // 62.0

//        // at 60 in/s
//        FollowerConstants.forwardZeroPowerAcceleration = -30.0000; //-36.90577841497325; // -30 for 30 in/s
//        FollowerConstants.lateralZeroPowerAcceleration = -80.0000; // -85.0
//
//        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0);
//        FollowerConstants.useSecondaryTranslationalPID = true;
//        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID
//
//        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.001,0);
//        FollowerConstants.useSecondaryHeadingPID = true;
//        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.001,0); // Not being used, @see useSecondaryHeadingPID
//
//        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.02,0,0.0002,0.6,0);
//        FollowerConstants.useSecondaryDrivePID = false;
//        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.02,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.xMovement = 78.80;
        FollowerConstants.yMovement = 62.0;

        // at 60 in/s
        FollowerConstants.forwardZeroPowerAcceleration = -30.0000; //-36.90577841497325; // -30 for 30 in/s
        FollowerConstants.lateralZeroPowerAcceleration = -80.0000; // -85.0

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.19,0,0.009,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.006,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.001,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.012,0,0.0012,0.6,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.01,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 6;
        FollowerConstants.centripetalScaling = 0.0004;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
