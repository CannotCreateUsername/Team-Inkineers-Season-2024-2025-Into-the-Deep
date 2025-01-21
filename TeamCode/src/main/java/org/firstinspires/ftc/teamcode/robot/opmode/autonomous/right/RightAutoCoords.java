package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.acmerobotics.roadrunner.Vector2d;

public class RightAutoCoords {
    // To make final adjustments quickly
//    public final double X_OFFSET = 0.8; Meet 1

    // VECTOR2D (FORWARD, SIDEWAYS)
    // NEGATIVE SIDEWAYS FOR RIGHT, POSITIVE FOR LEFT

    public double ROTATED = Math.toRadians(0);
    public double PICKUP = Math.toRadians(30);
    public double DROPOFF = Math.toRadians(-40);
    public double STRAIGHT = Math.toRadians(90);

    public Vector2d scoreSpecimenPos = new Vector2d(-6, 36);

    public Vector2d samplePos1 = new Vector2d(33, 26);
    public Vector2d samplePos2 = new Vector2d(43.5, 27.5);
    public Vector2d samplePos3 = new Vector2d(54, 26.5);
    public Vector2d sampleTransition = new Vector2d(38, 15);

    public Vector2d observationPos = new Vector2d(28, 16);
    public Vector2d specimenPickupPos = new Vector2d(28, 0.5);
}
