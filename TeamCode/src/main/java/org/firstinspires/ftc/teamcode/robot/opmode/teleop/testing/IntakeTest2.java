package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeTest2 extends LinearOpMode {
    private enum WristState {
        LEFT,
        RIGHT,
        CENTER
    }

    WristState wristState;

    @Override
    public void runOpMode() throws InterruptedException {

        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        wristState = WristState.CENTER;

        waitForStart();
        while (opModeIsActive()) {
            switch (wristState) {
                case CENTER:
                    wrist.setPosition(.5);
                    if (gamepadEx1.wasJustReleased(GamepadKeys.Button.A)) {
                        wristState = WristState.RIGHT;
                    } else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.B)) {
                        wristState = WristState.LEFT;
                    }
                    break;
                case LEFT:
                    wrist.setPosition(0);
                    if (gamepadEx1.wasJustReleased(GamepadKeys.Button.A)) {
                        wristState = WristState.RIGHT;
                    } else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.B)) {
                        wristState = WristState.CENTER;
                    }
                    break;
                case RIGHT:
                    wrist.setPosition(1);
                    if (gamepadEx1.wasJustReleased(GamepadKeys.Button.B)) {
                        wristState = WristState.LEFT;
                    } else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.A)) {
                        wristState = WristState.CENTER;
                    }
                    break;
            }

            if (gamepad1.right_trigger > 0) {
                intake.setPower(.8);
            } else {
                intake.setPower(0);
            }

            telemetry.addData("Wrist State", wristState);
            telemetry.update();
        }
    }
}
