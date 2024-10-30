package org.firstinspires.ftc.teamcode.robot.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystem;

@TeleOp(name = "Arm Reset", group = "Linear Opmode")
public class ArmReset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            armSubsystem.runReset(gamepad1);
            telemetry.update();
        }
    }
}
