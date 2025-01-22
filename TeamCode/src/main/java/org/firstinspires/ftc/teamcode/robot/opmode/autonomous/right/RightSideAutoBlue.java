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

        Action runPickupSamples = drive.actionBuilder(startPos)
                .setTangent(0)
                .splineToConstantHeading(coords.samplePos1.position, Math.toRadians(0))
                .build();

        Action runToIntake = drive.actionBuilder(coords.samplePos3)
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(coords.specimenPickupPos.position.x, coords.specimenPickupPos.position.y - 2), -Math.PI/2)
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
            Pose2d newScorePos = new Pose2d(coords.scoreSpecimenPos.position.x - i * 1.5, coords.scoreSpecimenPos.position.y, coords.ROTATED);

            Action runToNewScore = drive.actionBuilder(coords.specimenPickupPos)
                    .setTangent(Math.PI/2)
                    .splineToConstantHeading(newScorePos.position, Math.PI/2)
                    .build();

            Action runToPickUp = drive.actionBuilder(newScorePos)
//                    .strafeToLinearHeading(new Vector2d(newScorePos.x-8, newScorePos.y), coords.ROTATED)
                    .setTangent(-Math.PI/2)
                    .splineToConstantHeading(coords.specimenPickupPos.position, -Math.PI/2)
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
