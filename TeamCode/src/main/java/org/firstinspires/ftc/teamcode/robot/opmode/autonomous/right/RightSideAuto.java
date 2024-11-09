package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemAuto;

@Autonomous(name = "Your Only Auto", group = "Autonomous")
public class RightSideAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);

        ArmSubsystemAuto armSubsystem = new ArmSubsystemAuto();
        armSubsystem.init(hardwareMap, false);
        // Get coordinates to use
        RightAutoCoords coords = new RightAutoCoords();

        Action runToScore = drive.actionBuilder(startPos)
                .strafeToLinearHeading(coords.scoreSpecimenPos, coords.STRAIGHT)
                .build();
        Action runToSample = drive.actionBuilder(new Pose2d(coords.scoreSpecimenPos, coords.STRAIGHT))
                .strafeToLinearHeading(coords.backUpPos, coords.STRAIGHT)
                .strafeToLinearHeading(coords.samplePos1, coords.STRAIGHT)
                .build();
        Action runToObservation = drive.actionBuilder(new Pose2d(coords.samplePos1, coords.STRAIGHT))
                .strafeToLinearHeading(coords.observationPos, coords.ROTATED) // 2 Seconds Each?
                .waitSeconds(1)
                .strafeToLinearHeading(coords.samplePos2, coords.STRAIGHT)
                .waitSeconds(2)
                .strafeToLinearHeading(coords.observationPos, coords.ROTATED)
                .waitSeconds(1)
                .strafeToLinearHeading(coords.samplePos3, coords.STRAIGHT)
                .waitSeconds(2)
                .strafeToLinearHeading(coords.observationPos, coords.ROTATED)
                .build();

        telemetry.addLine("Ready! ^-^");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        armSubsystem.controlSlides(),
                        armSubsystem.controlWrist(),
                        new SequentialAction(
                                runToScore,
                                armSubsystem.score(),
                                // Slides down and move to sample at same time
                                new ParallelAction(
                                        armSubsystem.reset(),
                                        runToSample
                                ),
                                armSubsystem.spinIntake(2),

                                // Move robot based on time
                                new ParallelAction(
                                        runToObservation,
                                        new SequentialAction(
                                                new SleepAction(2),
                                                armSubsystem.spinOut(0.8),
                                                new SleepAction(2),
                                                armSubsystem.spinIntake(2),
                                                new SleepAction(2)
                                        )
                                )
                        )
                )
        );

    }
}
