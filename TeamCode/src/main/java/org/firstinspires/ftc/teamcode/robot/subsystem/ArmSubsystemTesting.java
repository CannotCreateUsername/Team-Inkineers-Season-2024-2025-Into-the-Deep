package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ArmSubsystemTesting extends ArmSubsystem {
    LinearOpMode opMode;

    public ArmSubsystemTesting (LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void runManualTesting(GamepadEx gamepad) {
        // Viper Slide Motion
        if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            setSlidePowers(1);
        } else if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            setSlidePowers(-1);
        } else {
            setSlidePowers(0);
        }

        // Worm Gear Motion
        if (gamepad.isDown(GamepadKeys.Button.Y)) {
            wormMotor.setPower(0.8);
        } else if (gamepad.isDown(GamepadKeys.Button.X)) {
            wormMotor.setPower(-0.8);
        } else {
            wormMotor.setPower(0);
        }
    }

    private boolean yeah = false;
    public void runV4BTesting(GamepadEx gamepad) {
        if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) {
            armDisplayText = "REST Position";
            setV4BPosition(ARM_REST_POS);
        } else if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
            armDisplayText = "LEFT Position";
            setV4BPosition(ARM_LEFT_POS);
        } else if (gamepad.wasJustPressed(GamepadKeys.Button.B)) {
            armDisplayText = "RIGHT Position";
            setV4BPosition(ARM_RIGHT_POS);
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
            armDisplayText = "Upper Testing";
            if (yeah) {
                setV4BPosition(V4B_LOWER_CENTER, V4B_UPPER_LEFT);
                yeah = false;
            } else {
                setV4BPosition(V4B_LOWER_CENTER, V4B_UPPER_RIGHT);
                yeah = true;
            }
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            armDisplayText = "Lower Testing Left";
            setV4BPosition(V4B_LOWER_LEFT, V4B_UPPER_CENTER);
        } else if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            armDisplayText = "Lower Testing Right";
            setV4BPosition(V4B_LOWER_RIGHT, V4B_UPPER_CENTER);
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.START)) {
            armDisplayText = "Mega Rest";
            setV4BPosition(MEGA_REST_POS);
        }
    }

    private boolean yeah2 = false;
    public void runSpecimenTesting(GamepadEx gamepad) {
        if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
            if (yeah2) {
                specimenWrist.setPosition(SPECIMEN_WRIST_OUTTAKE_ANGLE);
                specimenBar.setPosition(SPECIMEN_BAR_OUTTAKE_ANGLE);
                yeah2 = false;
            } else {
                specimenWrist.setPosition(SPECIMEN_WRIST_INTAKE_ANGLE);
                specimenBar.setPosition(SPECIMEN_BAR_INTAKE_ANGLE);
                yeah2 = true;
            }
        }

        opMode.telemetry.addData("Wrist Position", "um.");
    }

    public void runIntakeTesting(Gamepad gamepad) {
        if (gamepad.left_trigger > 0) {
            setIntakePowers(-1);
        } else if (gamepad.right_trigger > 0) {
            setIntakePowers(1);
        } else if (gamepad.left_bumper) {
            intakeServos.get(0).setPower(1);
        } else if (gamepad.right_bumper) {
            intakeServos.get(1).setPower(1);
        } else {
            setIntakePowers(0);
        }

        opMode.telemetry.addData("Test Intake", "Left/Right Trigger");
        opMode.telemetry.addData("Individual Servos", "Left Bumper for Left. Should Intake");
    }
}
