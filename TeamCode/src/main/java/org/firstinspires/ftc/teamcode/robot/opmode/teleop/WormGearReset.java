package org.firstinspires.ftc.teamcode.robot.opmode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Worm Reset", group = "Testing")
public class WormGearReset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepad = new GamepadEx(gamepad1);
        DcMotorEx arm_motor = hardwareMap.get(DcMotorEx.class, "worm_motor");

        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                arm_motor.setPower(-0.5);
            } else if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                arm_motor.setPower(0.5);
            } else {
                arm_motor.setPower(0);
            }
            gamepad.readButtons();
            telemetry.addData("Move Arm", "Right/Left Bumpers");
            telemetry.update();
        }
    }
}
