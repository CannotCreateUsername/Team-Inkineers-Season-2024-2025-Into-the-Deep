package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemTesting;

@TeleOp(name = "Intake Test M3", group = "Testing")
public class IntakeTestM3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ArmSubsystemTesting armSubsystem = new ArmSubsystemTesting(this);
        armSubsystem.initIntake(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            armSubsystem.runIntakeTesting(gamepad1);
            telemetry.update();
        }
    }
}
