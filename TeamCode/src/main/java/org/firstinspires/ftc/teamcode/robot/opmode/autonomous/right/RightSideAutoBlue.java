package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

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
                .waitSeconds(0.5)
                .strafeToLinearHeading(coords.waitForHumanPos, coords.ROTATED)
                .build();
        Action runToPickup = drive.actionBuilder(new Pose2d(coords.waitForHumanPos, coords.ROTATED))
                .strafeToLinearHeading(coords.specimenPickupPos, coords.ROTATED)
                .build();
        Action runRetry = drive.actionBuilder(new Pose2d(coords.specimenPickupPos, coords.ROTATED))
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
                                // Slides down and move to sample at same time
                                new ParallelAction(
                                        armSubsystem.reset(),
                                        runToSample
                                ),
                                armSubsystem.spinIn(2),

                                // Move 3 samples into observation zone based on time
                                new ParallelAction(
                                        runToObservation,
                                        new SequentialAction(
                                                new SleepAction(2),
                                                armSubsystem.spinOut(0.8),
                                                new SleepAction(2),
                                                armSubsystem.spinIn(2),
                                                new SleepAction(2),
                                                armSubsystem.spinOut(0.8),
                                                new SleepAction(2),
                                                armSubsystem.spinIn(2),
                                                new SleepAction(2),
                                                armSubsystem.spinOut(0.8)
                                        )
                                ),
                                armSubsystem.terminate()
                        )
                )
        );
        // Cycle Specimen
        for (int i = 1; i < 4; i++) {
            Actions.runBlocking(
                    new ParallelAction(
                            armSubsystem.controlActuators(),
                            runToPickup,
                            armSubsystem.pickUpSpecimen()
                    )
            );
            for (int e = 0; e < 2; e++) {
                // If intaked specimen, go to score
                if (armSubsystem.getSpecimenPickUp()) {
                    break;
                } else {
                    // Otherwise, try again to intake, back out for human player to adjust
                    Actions.runBlocking(
                            new ParallelAction(
                                    armSubsystem.controlActuators(),
                                    new SequentialAction(
                                            runRetry,
                                            new ParallelAction(
                                                    runToPickup,
                                                    armSubsystem.pickUpSpecimen()
                                            ),
                                            armSubsystem.terminate()
                                    )
                            )

                    );
                }
            }
            // Create action to score two inches to the left of previous scoring position
            // to avoid placing on top of scored specimen
            Vector2d newScorePos = new Vector2d(coords.scoreSpecimenPos.x, coords.scoreSpecimenPos.y + i*2);
            Action runToChamber = drive.actionBuilder(new Pose2d(coords.specimenPickupPos, coords.ROTATED))
                    .strafeToLinearHeading(newScorePos, coords.STRAIGHT)
                    .build();
            Action backToScore = drive.actionBuilder(new Pose2d(newScorePos, coords.STRAIGHT))
                    .strafeToLinearHeading(coords.waitForHumanPos, coords.ROTATED)
                    .build();
            Actions.runBlocking(
                    new ParallelAction (
                            armSubsystem.controlActuators(),
                            new SequentialAction(
                                    armSubsystem.readySpecimen(),
                                    runToChamber,
                                    armSubsystem.score(),
                                    backToScore,
                                    armSubsystem.terminate()
                            )
                    )
            );
        }
    }
}
