package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemAuto;

@Disabled
@Autonomous(name = "Right Auto +5", group = "Autonomous")
public class RightSideAutoBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Get coordinates to use
        RightAutoCoords coords = new RightAutoCoords();

        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);

        ArmSubsystemAuto armSubsystem = new ArmSubsystemAuto();
        armSubsystem.init(hardwareMap, false, true);

        Action runPickupSamples = drive.actionBuilder(startPos)
                .setTangent(0)
                .afterDisp(8, armSubsystem.pickUpAndDropOff())
                .splineToLinearHeading(coords.samplePos1, Math.toRadians(0))
                .strafeToLinearHeading(coords.samplePosBack.position, coords.DROPOFF)
                .afterDisp(4, armSubsystem.pickUpAndDropOff())
                .splineToLinearHeading(coords.samplePos2, Math.toRadians(0))
                .strafeToLinearHeading(coords.samplePosBack2.position, coords.DROPOFF)
                .afterDisp(4, armSubsystem.pickUpSample())
                .splineToLinearHeading(coords.samplePos3, Math.toRadians(0))
                .build();

        Action runToIntake = drive.actionBuilder(coords.samplePos3)
                .afterDisp(4, armSubsystem.readyIntake())
                .setTangent(-Math.PI)
                .splineToLinearHeading(new Pose2d(coords.specimenPickupPos.position.x, coords.specimenPickupPos.position.y-0.5, coords.ROTATED), -Math.PI/2)
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
                                        runPickupSamples
                                ),
                                new ParallelAction(
                                        runToIntake,
                                        new SequentialAction(
                                                new SleepAction(0.2),
                                                armSubsystem.dropOffSample2()
                                        )
                                ),
                                armSubsystem.terminate()
                        )
                )
        );

        for (int i = 0; i < 5; i++) {
            Pose2d newScorePos = new Pose2d(coords.scoreSpecimenPos.position.x - i * 1.5, coords.scoreSpecimenPos.position.y, coords.ROTATED);

            Action runToNewScore = drive.actionBuilder(coords.specimenPickupPos)
                    .setTangent(Math.PI/2)
                    .splineToConstantHeading(newScorePos.position, Math.PI/2)
                    .build();

            Action runToPickUp = drive.actionBuilder(newScorePos)
                    .setTangent(-Math.PI/2)
                    .splineToConstantHeading(coords.specimenPickupPos.position, -Math.PI/2)
                    .build();

            Actions.runBlocking(
                    new ParallelAction(
                            armSubsystem.controlActuators(),
                            new SequentialAction(
                                    new ParallelAction(
                                            new SequentialAction(
                                                    new SleepAction(0.1),
                                                    runToNewScore
                                            ),
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
