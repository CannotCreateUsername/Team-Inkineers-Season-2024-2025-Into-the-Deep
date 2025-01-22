package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class RightAutoCoords {
    // To make final adjustments quickly

    //      | +y
    // -----|=---> +x 0 radians
    //      |

    public double ROTATED = Math.toRadians(0);
    public double PICKUP = Math.toRadians(30);
    public double DROPOFF = Math.toRadians(-30);
//    public double STRAIGHT = Math.toRadians(90);

    public Pose2d scoreSpecimenPos = new Pose2d(-6, 36, ROTATED);

    public Pose2d samplePos1 = new Pose2d(33, 26, PICKUP);
    public Pose2d samplePos2 = new Pose2d(43.5, 27.5, PICKUP);
    public Pose2d samplePos3 = new Pose2d(54, 26.5, PICKUP);

    public Pose2d observationPos = new Pose2d(28, 16, ROTATED);
    public Pose2d specimenPickupPos = new Pose2d(28, 0.5, ROTATED);
}
