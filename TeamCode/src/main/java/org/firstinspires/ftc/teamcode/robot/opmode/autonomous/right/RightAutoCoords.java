package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.acmerobotics.roadrunner.Vector2d;

public class RightAutoCoords {
    // To make final adjustments quickly
//    public final double X_OFFSET = 0.8; Meet 1

    // VECTOR2D (FORWARD, SIDEWAYS)
    // NEGATIVE SIDEWAYS FOR RIGHT, POSITIVE FOR LEFT

    public double ROTATED = Math.toRadians(180);
    public double STRAIGHT = Math.toRadians(0);

    public Vector2d scoreSpecimenPos = new Vector2d(20, 5);

    public Vector2d samplePos1 = new Vector2d(20, -34);
    public Vector2d samplePos2 = new Vector2d(20, -46);
    public Vector2d samplePos3 = new Vector2d(20, -56);

    public Vector2d observationPos = new Vector2d(20, -38);
    public Vector2d specimenPickupPos = new Vector2d(10, -38);
}
