package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemTesting;

@TeleOp(name = "Specimen Arm Testing", group = "Testing")
public class SpecimenArmTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ArmSubsystemTesting armSubsystem = new ArmSubsystemTesting(this);
        armSubsystem.initSpecimen(hardwareMap);
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart();
        while (opModeIsActive()) {
            armSubsystem.runSpecimenTesting(gamepadEx1);

            gamepadEx1.readButtons();

            telemetry.update();
        }
    }
}
