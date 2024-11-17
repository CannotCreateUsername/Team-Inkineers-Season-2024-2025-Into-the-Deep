package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.acmerobotics.roadrunner.Vector2d;

public class RightAutoCoords {
    // To make final adjustments quickly
    public final double X_OFFSET = 0.8;

    // VECTOR2D (FORWARD, SIDEWAYS)
    // NEGATIVE SIDEWAYS FOR RIGHT, POSITIVE FOR LEFT

    public double ROTATED = Math.toRadians(180);
    public double STRAIGHT = Math.toRadians(0);

    public Vector2d scoreSpecimenPos = new Vector2d(25.8 - X_OFFSET, 5);
    public Vector2d backUpPos = new Vector2d(16 - X_OFFSET, -12);

    public Vector2d samplePos1 = new Vector2d(26 - X_OFFSET, -36);
    public Vector2d samplePos2 = new Vector2d(26 - X_OFFSET, -46);
    public Vector2d samplePos3 = new Vector2d(26 - X_OFFSET, -56);

    public Vector2d aroundSamplePos1 = new Vector2d(samplePos1.x+28 - X_OFFSET, samplePos1.y+4);
    public Vector2d aroundSamplePos2 = new Vector2d(samplePos2.x+28 - X_OFFSET, samplePos2.y+4);

    public Vector2d observationPos = new Vector2d(12 - X_OFFSET, -40);
    public Vector2d specimenPickupPos = new Vector2d(32 - X_OFFSET, -36);
    public Vector2d waitForHumanPos = new Vector2d(40 - X_OFFSET, -36);
}
