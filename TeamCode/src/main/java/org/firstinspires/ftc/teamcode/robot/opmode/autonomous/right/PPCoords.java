package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.pedropathing.localization.Pose;

public class PPCoords {

    public double ROTATED = Math.toRadians(-90);
    public double STRAIGHT = Math.toRadians(0);

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>

    /** Start Pose of our robot */
    public final Pose startPose = new Pose(10, 66, STRAIGHT);

    // From Start Pose
    public final Pose push1Pose = new Pose(54, 28);
    public final Pose controlPush1 = new Pose(10, 20);
    public final Pose controlPush12 = new Pose(60, 48);
    public final Pose observationPose1 = new Pose(26, 26);
    public final Pose controlObservationPose1 = new Pose(observationPose1.getX() + 4, observationPose1.getY() - 8);

    public final Pose push2Pose = new Pose(56, 18);
    public final Pose controlPush2 = new Pose(observationPose1.getX() + 10, observationPose1.getY()+12);
    public final Pose controlPush22 = new Pose(push2Pose.getX(), observationPose1.getY()+12);
    public final Pose observationPose2 = new Pose(26, 16);
    public final Pose controlObservationPose2 = new Pose(observationPose2.getX() + 4, observationPose2.getY() - 8);

    public final Pose push3Pose = new Pose(60, 13);
    public final Pose controlPush3 = new Pose(observationPose2.getX() + 10, observationPose2.getY()+10);
    public final Pose controlPush32 = new Pose(push3Pose.getX(), observationPose2.getY()+10);
    public final Pose observationPose3 = new Pose(26, 12);

    public final Pose pickupSpecimenPose = new Pose(10, 37, ROTATED); // 0.5 in to the left of hole
    public final Pose controlSpecimen0 = new Pose(pickupSpecimenPose.getX() + 10, pickupSpecimenPose.getY() + 2); // Rotate -90 Degrees

    public final Pose scorePose = new Pose(41.5, 66+9, ROTATED); // 41.5, 10
}
