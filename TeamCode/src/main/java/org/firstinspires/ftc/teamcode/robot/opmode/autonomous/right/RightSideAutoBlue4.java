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
import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemAuto;

@Autonomous(name = "Right Auto +4", group = "Autonomous")
public class RightSideAutoBlue4 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Get coordinates to use
        RightAutoCoords coords = new RightAutoCoords();

        Pose2d startPos = new Pose2d(0, 0, coords.ROTATED);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);

        ArmSubsystemAuto armSubsystem = new ArmSubsystemAuto();
        armSubsystem.init(hardwareMap, false);

        Action runPickupSamples = drive.actionBuilder(startPos)
                .setTangent(0)
                .splineToLinearHeading(coords.samplePos1Push, Math.PI/3)
                .strafeToConstantHeading(coords.observationPos.position)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(coords.samplePos2Push, 0)
                .strafeToConstantHeading(coords.observationPos2.position)
                .build();

        Action runToIntake = drive.actionBuilder(coords.observationPos2)
                .afterDisp(0, armSubsystem.readyIntake())
                .setTangent(-Math.PI)
                .strafeToConstantHeading(coords.specimenPickupPos.position)
                .build();

        telemetry.addLine("Wait for wrist! ^-^");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        armSubsystem.controlActuators(),
                        new SequentialAction(
                                armSubsystem.moveSpecimen(ArmSubsystem.SpecimenState.HANG),
                                new ParallelAction(
                                        runPickupSamples
                                ),
                                new ParallelAction(
                                        runToIntake
                                ),
                                armSubsystem.terminate()
                        )
                )
        );
        int cycles = 4;
        for (int i = 0; i < cycles; i++) {
            boolean ending = i == cycles-1;
            Pose2d newScorePos = new Pose2d(coords.scoreSpecimenPos.position.x - i * 2.5, coords.scoreSpecimenPos.position.y, coords.ROTATED);

            Action runToNewScore = drive.actionBuilder(coords.specimenPickupPos)
                    .setTangent(Math.PI/2)
                    .splineToConstantHeading(newScorePos.position, Math.PI/2)
                    .build();

            Action runToPickUp = drive.actionBuilder(newScorePos)
                    .setTangent(-Math.PI/2)
                    .splineToLinearHeading(coords.specimenPickupPos, ending ? -Math.PI/3 : -Math.PI/2)
                    .build();

            Actions.runBlocking(
                    new ParallelAction(
                            armSubsystem.controlActuators(),
                            new SequentialAction(
                                    new ParallelAction(
                                            new SequentialAction(
                                                    new SleepAction(0.2),
                                                    runToNewScore
                                            ),
                                            armSubsystem.readySpecimen()
                                    ),
                                    armSubsystem.hangSpecimenTransition(runToPickUp, ending),
                                    armSubsystem.terminate()
                            )
                    )
            );
        }
    }
}
