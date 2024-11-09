package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.acmerobotics.roadrunner.Vector2d;

public class RightAutoCoords {
    // VECTOR2D (FORWARD, SIDEWAYS)
    // NEGATIVE SIDEWAYS FOR RIGHT, POSITIVE FOR LEFT

    public double ROTATED = Math.toRadians(180);
    public double STRAIGHT = Math.toRadians(0);

    public Vector2d scoreSpecimenPos = new Vector2d(32, 5);
    public Vector2d backUpPos = new Vector2d(26, 5);

    public Vector2d samplePos1 = new Vector2d(36, -36);
    public Vector2d samplePos2 = new Vector2d(36, -46);
    public Vector2d samplePos3 = new Vector2d(36, -56);

    public Vector2d observationPos = new Vector2d(24, -46);
}
