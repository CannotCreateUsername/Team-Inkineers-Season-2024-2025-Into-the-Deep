package org.firstinspires.ftc.teamcode.robot.opmode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemTeleOp;

@TeleOp(name = "Arm/Hang Testing", group = "Testing")
public class WormGearReset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepad = new GamepadEx(gamepad1);
        ArmSubsystemTeleOp arm = new ArmSubsystemTeleOp();
        arm.initManualTesting(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            arm.runManualTesting(gamepad);

            gamepad.readButtons();
            telemetry.addData("Move Arm", "Right/Left Bumpers");
            telemetry.addData("Arm Telemetry", arm.slideDisplayText);
            telemetry.addData("Slides Telemetry", arm.slideDisplayText);
            telemetry.addData("Hang Telemetry", arm.getHangPosition());
            telemetry.update();
        }
    }
}
