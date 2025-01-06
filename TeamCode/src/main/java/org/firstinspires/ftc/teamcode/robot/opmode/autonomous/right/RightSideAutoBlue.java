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
                .strafeToLinearHeading(coords.scoreSpecimenPos, coords.ROTATED)
                .build();

        Action runToSample1 = drive.actionBuilder(new Pose2d(coords.scoreSpecimenPos, coords.ROTATED))
                .strafeToLinearHeading(coords.backScorePos, coords.ROTATED)
                .strafeToLinearHeading(coords.samplePos1, coords.ROTATED)
                .build();

        Action runToSample2 = drive.actionBuilder(new Pose2d(coords.samplePos1, coords.ROTATED))
                .strafeToLinearHeading(coords.samplePos2, coords.ROTATED)
                .build();

        Action runToDropOff = drive.actionBuilder(new Pose2d(coords.samplePos2, coords.ROTATED))
                .strafeToLinearHeading(coords.observationPos, coords.ROTATED)
                .build();

        Action runToIntake = drive.actionBuilder(new Pose2d(coords.observationPos, coords.ROTATED))
                .strafeToLinearHeading(coords.specimenPickupPos, coords.ROTATED)
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
                                armSubsystem.hangSpecimenTransition(runToSample1),
                                armSubsystem.pickUpSample(false),
                                armSubsystem.dropOffSample(),
                                runToSample2,
                                armSubsystem.pickUpSample(true),
                                runToDropOff,
                                armSubsystem.dropOffSample(),
                                armSubsystem.readyIntake(),
                                runToIntake,
                                armSubsystem.terminate()
                        )
                )
        );

        for (int i = 1; i <= 3; i++) {
            Vector2d newScorePos = new Vector2d(coords.scoreSpecimenPos.x - i * 2, coords.scoreSpecimenPos.y);

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
                                    armSubsystem.hangSpecimenTransition(runToPickUp),
                                    armSubsystem.terminate()
                            )
                    )
            );
        }
    }
}
