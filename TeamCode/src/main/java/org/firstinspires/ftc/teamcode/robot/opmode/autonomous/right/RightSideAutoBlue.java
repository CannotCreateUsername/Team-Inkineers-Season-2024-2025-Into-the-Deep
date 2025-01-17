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


        // TODO: Fix everything
        Action runToScore = drive.actionBuilder(startPos)
                .strafeToLinearHeading(coords.scoreSpecimenPos, coords.ROTATED)
                .build();

        Action runToSample1 = drive.actionBuilder(new Pose2d(coords.scoreSpecimenPos, coords.ROTATED))
                .setTangent(-Math.PI/2)
                .splineToConstantHeading(coords.samplePos1, 0)
                .build();

        Action runToSample2 = drive.actionBuilder(new Pose2d(coords.samplePos1, coords.ROTATED))
                .strafeToLinearHeading(coords.sampleTransition, coords.ROTATED)
                .waitSeconds(0.5)
                .setTangent(0)
                .splineToConstantHeading(coords.samplePos2, Math.PI/2)
                .build();

        Action runToIntake = drive.actionBuilder(new Pose2d(coords.samplePos2, coords.ROTATED))
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(coords.specimenPickupPos.x, coords.specimenPickupPos.y - 2), -Math.PI/2)
                .build();

        telemetry.addLine("Wait for wrist! ^-^");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        armSubsystem.controlActuators(),
                        new SequentialAction(
                                new ParallelAction(
                                        runToScore,
                                        armSubsystem.readySpecimen()
                                ),
                                armSubsystem.hangSpecimenTransition(runToSample1, true),
                                new ParallelAction(
                                        armSubsystem.dropOffSample(),
                                        runToSample2,
                                        new SequentialAction(
                                                new SleepAction(1.4),
                                                armSubsystem.pickUpSample(false)
                                        )
                                ),
                                armSubsystem.readyIntake(),
                                new ParallelAction(
                                        runToIntake,
                                        armSubsystem.dropOffSample()
                                ),
                                armSubsystem.terminate()
                        )
                )
        );

        for (int i = 1; i <= 3; i++) {
            Vector2d newScorePos = new Vector2d(coords.scoreSpecimenPos.x - i * 1.5, coords.scoreSpecimenPos.y);

            Action runToNewScore = drive.actionBuilder(new Pose2d(coords.specimenPickupPos, coords.ROTATED))
                    .setTangent(Math.PI/2)
                    .splineToConstantHeading(newScorePos, Math.PI/2)
                    .build();

            Action runToPickUp = drive.actionBuilder(new Pose2d(newScorePos, coords.ROTATED))
//                    .strafeToLinearHeading(new Vector2d(newScorePos.x-8, newScorePos.y), coords.ROTATED)
                    .setTangent(-Math.PI/2)
                    .splineToConstantHeading(coords.specimenPickupPos, -Math.PI/2)
                    .build();

            Actions.runBlocking(
                    new ParallelAction(
                            armSubsystem.controlActuators(),
                            new SequentialAction(
                                    new ParallelAction(
                                            runToNewScore,
                                            armSubsystem.readySpecimen()
                                    ),
                                    armSubsystem.hangSpecimenTransition(runToPickUp, false),
                                    armSubsystem.terminate()
                            )
                    )
            );
        }
    }
}
