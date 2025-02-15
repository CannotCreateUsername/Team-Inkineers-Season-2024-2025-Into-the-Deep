package org.firstinspires.ftc.teamcode.robot.opmode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemTesting;

@TeleOp(name = "Arm/Hang Testing", group = "Testing")
public class WormGearReset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepad = new GamepadEx(gamepad1);
        ArmSubsystemTesting arm = new ArmSubsystemTesting(this);
        arm.initManualTesting(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            arm.runManualTesting(gamepad);

            gamepad.readButtons();
            telemetry.addData("Arm Telemetry", arm.armDisplayText);
            telemetry.addData("Slides Telemetry", arm.slideDisplayText);
            telemetry.addLine();
            telemetry.addData("Move Worm Gear", "Y/X");
            telemetry.addData("Worm Gear Position", arm.wormMotor.getCurrentPosition());
            telemetry.addData("Worm Gear Target", arm.wormMotor.getTargetPosition());
            telemetry.addLine();
            telemetry.addData("Move Slides", "Right/Left Bumpers");
            telemetry.addData("Slides Position", arm.getSlidesPosition());
            telemetry.addData("Slides Target", arm.targetSlidePosition);
            telemetry.addLine();
            telemetry.addData("Move Linear Actuator", "Up/Down DPAD");
            telemetry.addData("Linear Actuator Position", arm.hangMotor.getCurrentPosition());
            telemetry.addData("Linear Actuator Target", arm.hangMotor.getTargetPosition());
            telemetry.update();
        }
    }
}
