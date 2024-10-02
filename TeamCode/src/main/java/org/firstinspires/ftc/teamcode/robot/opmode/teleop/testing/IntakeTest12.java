package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "#12 Intake Test", group = "Testing")
public class IntakeTest12 extends LinearOpMode {
    private enum ExtendState {
        REST,
        EXTENDED
    }

    /** @noinspection FieldCanBeLocal*/
    private final double REST_POSITION = 0;
    /** @noinspection FieldCanBeLocal*/
    private final double EXTEND_POSITION = 1;

    ExtendState extendState;

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        Servo extender = hardwareMap.get(Servo.class, "extender");

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        extender.setPosition(0);
        extendState = ExtendState.REST;

        waitForStart();
        while (opModeIsActive()) {

            // Extend/retract intake using button A.
            switch (extendState) {
                case REST:
                    extender.setPosition(REST_POSITION);
                    if (gamepadEx1.wasJustReleased(GamepadKeys.Button.A)) {
                        extendState = ExtendState.EXTENDED;
                    }
                    break;
                case EXTENDED:
                    extender.setPosition(EXTEND_POSITION);
                    if (gamepadEx1.wasJustReleased(GamepadKeys.Button.A)) {
                        extendState = ExtendState.REST;
                    }
                    break;
            }
        }

        // Spins IN when right trigger is held, and OUT if left trigger is held. If both are down at the same time, the power should cancel out to be zero.
        intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        telemetry.addData("Extension State", extendState);
        telemetry.update();
    }
}
