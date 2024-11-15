package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemAuto;

@Autonomous(name = "Blue Auto", group = "Autonomous")
public class RightSideAutoBlue extends LinearOpMode {

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
                .strafeToLinearHeading(coords.aroundSamplePos1, coords.STRAIGHT)
                .strafeToLinearHeading(coords.observationPos, coords.STRAIGHT)
                .strafeToLinearHeading(coords.aroundSamplePos1, coords.STRAIGHT)
                .strafeToLinearHeading(coords.aroundSamplePos2, coords.STRAIGHT)
                .strafeToLinearHeading(coords.observationPos, coords.STRAIGHT)
                .strafeToLinearHeading(coords.waitForHumanPos, coords.ROTATED)
                .waitSeconds(1)
                .build();

        telemetry.addLine("Ready! ^-^");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        armSubsystem.controlActuators(),
                        new SequentialAction(
                                runToScore,
                                armSubsystem.score(),
                                // Reset and start going
                                new ParallelAction(
                                        armSubsystem.slidesReset(false),
                                        runToSample
                                ),
                                armSubsystem.terminate()
                        )
                )
        );
        // Cycle Specimen
        for (int i = 1; i <= 2; i++) {
            Action runToPickup = drive.actionBuilder(new Pose2d(coords.waitForHumanPos, coords.ROTATED))
                    .strafeToLinearHeading(coords.specimenPickupPos, coords.ROTATED)
                    .build();
            Actions.runBlocking(
                    new ParallelAction(
                            armSubsystem.controlActuators(),
                            runToPickup,
                            new SequentialAction(
                                    armSubsystem.pickUpSpecimen(1),
                                    armSubsystem.terminate()
                            )
                    )
            );
            // Offset two inches to the left of previous scoring position
            // to avoid placing on top of scored specimen. Also +.5 in for padding
            Vector2d newScorePos = new Vector2d(coords.scoreSpecimenPos.x+1, coords.scoreSpecimenPos.y + i*2);
            Action runToChamber = drive.actionBuilder(new Pose2d(coords.specimenPickupPos, coords.ROTATED))
                    .strafeToLinearHeading(newScorePos, coords.STRAIGHT)
                    .build();
            Action backToScore = drive.actionBuilder(new Pose2d(newScorePos, coords.STRAIGHT))
                    .strafeToLinearHeading(coords.waitForHumanPos, coords.ROTATED)
                    .waitSeconds(0.5)
                    .build();
            Action park = drive.actionBuilder(new Pose2d(newScorePos, coords.STRAIGHT))
                    .strafeToLinearHeading(new Vector2d(coords.observationPos.x-4, coords.observationPos.y), coords.STRAIGHT)
                    .build();
            if (i < 2) {
                Actions.runBlocking(
                        new ParallelAction (
                                armSubsystem.controlActuators(),
                                new SequentialAction(
                                        armSubsystem.readySpecimen(),
                                        runToChamber,
                                        armSubsystem.score(),
                                        new ParallelAction(
                                                armSubsystem.slidesReset(false),
                                                backToScore
                                        ),
                                        armSubsystem.terminate()
                                )
                        )
                );
            } else {
                Actions.runBlocking(
                        new ParallelAction (
                                armSubsystem.controlActuators(),
                                new SequentialAction(
                                        armSubsystem.readySpecimen(),
                                        runToChamber,
                                        armSubsystem.score(),
                                        new ParallelAction(
                                                armSubsystem.slidesReset(true),
                                                park
                                        ),
                                        armSubsystem.terminate()
                                )
                        )
                );
            }
        }
    }
}
