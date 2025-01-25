package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class RightAutoCoords {
    // To make final adjustments quickly

    //      | +y
    // -----|=---> +x 0 radians
    //      |

    public double ROTATED = Math.toRadians(0);
    public double PICKUP = Math.toRadians(60);
    public double DROPOFF = Math.toRadians(-30);
    public double STRAIGHT = Math.toRadians(90);

    public Pose2d scoreSpecimenPos = new Pose2d(-11, 35, ROTATED);

    // OLD
    public Pose2d samplePos1 = new Pose2d(34, 20, PICKUP);
    public Pose2d samplePosBack = new Pose2d(25, 16, DROPOFF);
    public Pose2d samplePos2 = new Pose2d(41, 20, PICKUP);
    public Pose2d samplePosBack2 = new Pose2d(33, 14, DROPOFF);
    public Pose2d samplePos3 = new Pose2d(48, 20, PICKUP);

    public Pose2d samplePos1Push = new Pose2d(34, 50, ROTATED);
    public Pose2d samplePos2Push = new Pose2d(40, 50, ROTATED);
    public Pose2d samplePos3Push = new Pose2d(46, 50, STRAIGHT);

    public Pose2d observationPos = new Pose2d(32, 12, ROTATED);
    public Pose2d observationPos2 = new Pose2d(38, 12, ROTATED);
    public Pose2d observationPos3 = new Pose2d(46, 14, STRAIGHT);
    public Pose2d specimenPickupPos = new Pose2d(20, -0.4, ROTATED);
}
