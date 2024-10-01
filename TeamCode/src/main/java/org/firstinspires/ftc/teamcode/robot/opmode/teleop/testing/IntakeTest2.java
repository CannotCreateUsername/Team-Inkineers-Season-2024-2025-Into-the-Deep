package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "#2 Intake Test", group = "Testing")
public class IntakeTest2 extends LinearOpMode {
    private enum WristState {
        LEFT,
        RIGHT,
        CENTER
    }

    /** @noinspection FieldCanBeLocal*/
    private final double LEFT_POSITION = 0;
    /** @noinspection FieldCanBeLocal*/
    private final double CENTER_POSITION = 0.5;
    /** @noinspection FieldCanBeLocal*/
    private final double RIGHT_POSITION = 1;

    WristState wristState;

    @Override
    public void runOpMode() throws InterruptedException {

        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        wrist.setPosition(.5);
        wristState = WristState.CENTER;

        waitForStart();
        while (opModeIsActive()) {

            // Run the wrist. Button A to toggle between rest and rotated RIGHT, button B to toggle between rest and rotated LEFT.
            switch (wristState) {
                case CENTER:
                    wrist.setPosition(CENTER_POSITION);
                    if (gamepadEx1.wasJustReleased(GamepadKeys.Button.A)) {
                        wristState = WristState.RIGHT;
                    } else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.B)) {
                        wristState = WristState.LEFT;
                    }
                    break;
                case LEFT:
                    wrist.setPosition(LEFT_POSITION);
                    if (gamepadEx1.wasJustReleased(GamepadKeys.Button.A)) {
                        wristState = WristState.RIGHT;
                    } else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.B)) {
                        wristState = WristState.CENTER;
                    }
                    break;
                case RIGHT:
                    wrist.setPosition(RIGHT_POSITION);
                    if (gamepadEx1.wasJustReleased(GamepadKeys.Button.B)) {
                        wristState = WristState.LEFT;
                    } else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.A)) {
                        wristState = WristState.CENTER;
                    }
                    break;
            }

            // Spins IN when right trigger is held, and OUT if left trigger is held. If both are down at the same time, the power should cancel out to be zero.
            intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            telemetry.addData("Wrist State", wristState);
            telemetry.update();
        }
    }
}
