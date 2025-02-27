package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.pedropathing.localization.Pose;

public class RightAutoCoordsPP {

    public double ROTATED = Math.toRadians(0);
    public double STRAIGHT = Math.toRadians(90);

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>

    /** Start Pose of our robot */
    public final Pose startPose = new Pose(0, 0, STRAIGHT);

    public final Pose scorePose1 = new Pose(-11, 35, ROTATED);
    public final Pose scorePose2 = new Pose(-13, 35, ROTATED);
    public final Pose scorePose3 = new Pose(-15, 35, ROTATED);
    public final Pose scorePose4 = new Pose(-17, 35, ROTATED);
    public final Pose scorePose5 = new Pose(-19, 35, ROTATED);

    public final Pose specimenPickupPose = new Pose(20, 0, ROTATED);

    public final Pose push1Pose = new Pose(33, 52, STRAIGHT);
    public final Pose push2Pose = new Pose(39, 52, STRAIGHT);
    public final Pose push3Pose = new Pose(45, 52, STRAIGHT);

    public final Pose observationPos1 = new Pose(32, 12, STRAIGHT);
    public final Pose observationPos2 = new Pose(39, 12, STRAIGHT);
    public final Pose observationPos3 = new Pose(45, 12, STRAIGHT);

    public final Pose parkPose = new Pose(60, 98, ROTATED);

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    public final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));
}
