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
import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemAuto;

@Autonomous(name = "Right Auto", group = "Autonomous")
public class RightSideAutoBlue extends LinearOpMode {

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
                .strafeToLinearHeading(coords.scoreSpecimenPos, coords.ROTATED)
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
                                armSubsystem.scoreAndTransitionToPickup(false),
                                armSubsystem.terminate()
                        )
                )
        );
        for (int i = 0; i < 2; i++) {
            drive.updatePoseEstimate();
            Action runToPickup = drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(coords.samplePos1.x, coords.samplePos1.y-(10*i)), coords.ROTATED)
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

        for (int i = 1; i < 2; i++) {
            drive.updatePoseEstimate();
            Action runToObservation = drive.actionBuilder(drive.pose)
                    .waitSeconds(0.5)
                    .strafeToLinearHeading(coords.specimenPickupPos, coords.ROTATED)
                    .build();
            Vector2d newScorePos = new Vector2d(coords.scoreSpecimenPos.x-3.5, coords.scoreSpecimenPos.y+i*4);
            Action runToScore2 = drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(newScorePos, coords.ROTATED)
                    .build();
            Action runToPickup2 = drive.actionBuilder(new Pose2d(newScorePos, coords.ROTATED))
                    .strafeToLinearHeading(coords.observationPos, coords.ROTATED)
                    .build();
            Actions.runBlocking(
                    new ParallelAction(
                            armSubsystem.controlActuators(),
                            new SequentialAction(
                                    new ParallelAction(
                                            armSubsystem.transitionToScore(runToObservation),
                                            new SequentialAction(
                                                    new SleepAction(1.7),
                                                    runToScore2
                                            )
                                    ),
                                    i == 1 ? armSubsystem.scoreAndTransitionToPickup(true) : armSubsystem.scoreAndTransitionToPickup(false),
                                    new ParallelAction(
                                            runToPickup2,
                                            armSubsystem.moveArm(ArmSubsystem.ArmState.RIGHT_FAR)
                                    ),
                                    armSubsystem.terminate()
                            )
                    )
            );

        }
        drive.updatePoseEstimate();
        Action runToPark = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(coords.specimenPickupPos, coords.ROTATED)
                .build();
        Actions.runBlocking(runToPark);
    }
}
