package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

public class GamepadHelper {

    public GamepadHelper() {
        gameStickTimer = new ElapsedTime();
    }

    private final double MIN_MULTIPLIER = 0.4;
    private final double  MAX_MULTIPLIER = 1.00; // Runs at 100% at full ramp
    private final double incrementMultiplier = 0.22;
    private final double timeIncrementInMs = 200;

    private double gameStickMultiplier = MIN_MULTIPLIER;

    ElapsedTime gameStickTimer;

    public double getRampingValue(float gameStick) {
        if (gameStick != 0) {
            // Increment every 200 milliseconds
            if (gameStickTimer.milliseconds() > timeIncrementInMs) {
                gameStickMultiplier += incrementMultiplier;
                gameStickTimer.reset();
            }

            // Does not go past limit
            if (gameStickMultiplier > MAX_MULTIPLIER) {
                gameStickMultiplier = MAX_MULTIPLIER;
            }
        } else {
            gameStickMultiplier = MIN_MULTIPLIER;
            gameStickTimer.reset();
        }
        return gameStick*gameStickMultiplier;
    }
}
