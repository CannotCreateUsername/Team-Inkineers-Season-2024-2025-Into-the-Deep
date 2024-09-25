package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystem.IntakeSubsystem;

abstract class RightSideAuto extends LinearOpMode {

    IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);
    ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap);

    void run() {
        // Put Code Here for autonomous
    }

    @Override
    public void runOpMode() throws InterruptedException {
        run();
    }
}
