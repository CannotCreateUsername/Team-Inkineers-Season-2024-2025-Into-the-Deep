package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemTesting;

public class SpecimenArmTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ArmSubsystemTesting armSubsystem = new ArmSubsystemTesting();
        armSubsystem.initSpecimen(hardwareMap);
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart();
        while (opModeIsActive()) {
            armSubsystem.runSpecimenTesting(gamepadEx1, this);
            telemetry.update();
        }
    }
}
