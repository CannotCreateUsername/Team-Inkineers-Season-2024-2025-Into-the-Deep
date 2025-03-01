package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.pedropathing.localization.Pose;

public class RightAutoCoordsPP {

    public double ROTATED = Math.toRadians(-90);
    public double STRAIGHT = Math.toRadians(0);

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>

    /** Start Pose of our robot */
    public final Pose startPose = new Pose(8, 66, STRAIGHT);

    // From Start Pose
    public final Pose push1Pose = new Pose(60, 26);
    public final Pose controlPush1 = new Pose(10, 20);
    public final Pose controlPush12 = new Pose(60, 48);

    public final Pose push2Pose = new Pose(60, 16);
    public final Pose controlPush2 = new Pose(push2Pose.getX() + 2, push2Pose.getY()+12);
    public final Pose push3Pose = new Pose(60, 8);
    public final Pose controlPush3 = new Pose(push3Pose.getX() + 2, push3Pose.getY()+12);

    public final Pose observationPose1 = new Pose(21, 26);
    public final Pose observationPose2 = new Pose(21, 16);
    public final Pose observationPose3 = new Pose(21, 8);

    public final Pose pickupSpecimenPose = new Pose(8, 30, ROTATED);
    public final Pose controlSpecimen0 = new Pose(18, 32); // Rotate -90 Degrees

    public final Pose scorePose0 = new Pose(40, 68, ROTATED);
}
