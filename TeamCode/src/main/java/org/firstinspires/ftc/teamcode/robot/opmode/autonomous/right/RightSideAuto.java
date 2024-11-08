package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.robot.subsystem.meet0.IntakeSubsystem0;

abstract class RightSideAuto extends LinearOpMode {

    IntakeSubsystem0 intakeSubsystem = new IntakeSubsystem0(hardwareMap);
    TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

    Action runToChamber = drive.actionBuilder(new Pose2d(0, 0, 0))
            .lineToY(20)
            .build();

    void run() {
        // Put Code Here for autonomous
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Add initialization code here
        waitForStart();
        run();
    }
}
