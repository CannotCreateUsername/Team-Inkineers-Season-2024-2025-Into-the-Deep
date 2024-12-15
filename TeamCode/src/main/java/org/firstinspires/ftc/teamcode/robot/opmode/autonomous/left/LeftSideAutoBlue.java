package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.left;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right.RightAutoCoords;
import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemAuto;

@Autonomous(name = "Left Auto", group = "Autonomous")
public class LeftSideAutoBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);

        ArmSubsystemAuto armSubsystem = new ArmSubsystemAuto();
        armSubsystem.init(hardwareMap, false);
        // Get coordinates to use
        RightAutoCoords coords = new RightAutoCoords();

        Action runToScore = drive.actionBuilder(startPos)
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(coords.scoreSpecimenPos.x, coords.scoreSpecimenPos.y-14), coords.ROTATED)
                .build();

        telemetry.addLine("Wait for wrist! ^-^");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        armSubsystem.controlActuators(),
                        new SequentialAction(
                                new SleepAction(4),
                                new ParallelAction(
                                        runToScore,
                                        armSubsystem.readySpecimen()
                                ),
                                armSubsystem.scoreAndTransitionToPickup(true),
                                armSubsystem.terminate()
                        )
                )
        );
        drive.updatePoseEstimate();
        Action runToPark = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(coords.leftParkPos, coords.ROTATED)
                .build();
        Actions.runBlocking(runToPark);
    }
}
