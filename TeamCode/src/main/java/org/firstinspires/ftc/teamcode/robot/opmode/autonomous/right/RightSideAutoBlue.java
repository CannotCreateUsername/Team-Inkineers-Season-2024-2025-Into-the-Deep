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

@Autonomous(name = "Right Auto", group = "Autonomous")
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

        telemetry.addLine("Wait for wrist! ^-^");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        armSubsystem.controlActuators(),
                        new SequentialAction(
                                runToScore,
                                armSubsystem.score(),
                                armSubsystem.scoreAndTransitionToPickup(false),
                                armSubsystem.terminate()
                        )
                )
        );
        for (int i = 0; i < 2; i++) {
            Action runToPickup = drive.actionBuilder(new Pose2d(coords.samplePos1, coords.STRAIGHT))
                    .strafeToLinearHeading(new Vector2d(coords.samplePos1.x-(16*i), coords.samplePos1.y), coords.STRAIGHT)
                    .build();

            Actions.runBlocking(
                    new ParallelAction(
                            armSubsystem.controlActuators(),
                            new SequentialAction(
                                    runToPickup,
                                    armSubsystem.pickUpAndDropOff(),
                                    armSubsystem.terminate()
                            )
                    )

            );
        }

        Action runToObservation = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(coords.observationPos, coords.STRAIGHT)
                .strafeToLinearHeading(coords.specimenPickupPos, coords.STRAIGHT)
                .build();
        Actions.runBlocking(runToObservation);

        for (int i = 1; i < 3; i++) {
            Vector2d newScorePos = new Vector2d(coords.scoreSpecimenPos.x, coords.scoreSpecimenPos.y+i*2);
            Action runToScore2 = drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(newScorePos, coords.STRAIGHT)
                    .build();
            Action runToPickup2 = drive.actionBuilder(new Pose2d(newScorePos, coords.STRAIGHT))
                    .strafeToLinearHeading(coords.specimenPickupPos, coords.STRAIGHT)
                    .build();
            Actions.runBlocking(
                    new ParallelAction(
                            armSubsystem.controlActuators(),
                            new SequentialAction(
                                    new ParallelAction(
                                            armSubsystem.transitionToScore(),
                                            new SequentialAction(
                                                    new SleepAction(1),
                                                    runToScore2
                                            )
                                    ),
                                    i == 2 ? armSubsystem.scoreAndTransitionToPickup(true) : armSubsystem.scoreAndTransitionToPickup(false),
                                    runToPickup2,
                                    armSubsystem.terminate()
                            )
                    )
            );

        }
    }
}
