package org.firstinspires.ftc.teamcode.robot.opmode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Arm/Hang Reset", group = "Testing")
public class WormGearReset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepad = new GamepadEx(gamepad1);
        DcMotorEx arm_motor = hardwareMap.get(DcMotorEx.class, "worm_motor");
        DcMotor hangMotor = hardwareMap.get(DcMotor.class, "hang_motor");

        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                arm_motor.setPower(-0.5);
            } else if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                arm_motor.setPower(0.5);
            } else {
                arm_motor.setPower(0);
            }
            if (gamepad.isDown(GamepadKeys.Button.DPAD_UP)) {
                hangMotor.setPower(1);
            } else if (gamepad.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                hangMotor.setPower(-1);
            } else {
                hangMotor.setPower(0);
            }
            if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            gamepad.readButtons();
            telemetry.addData("Move Arm", "Right/Left Bumpers");
            telemetry.addData("Arm Position", arm_motor.getCurrentPosition());
            telemetry.addData("Hang Position", hangMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
