package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

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

@Autonomous(name = "Swiggly Turn Test", group = "Test")
public class SplineTurnTest extends LinearOpMode {

    Pose2d path1 = new Pose2d(10, 20, Math.toRadians(30));
    Pose2d path2 = new Pose2d(20, 20, Math.toRadians(30));
    Pose2d path3 = new Pose2d(30, 20, Math.toRadians(30));
    Pose2d path4 = new Pose2d(40, 20, Math.toRadians(30));

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);

        ArmSubsystemAuto armSubsystem = new ArmSubsystemAuto();
        armSubsystem.init(hardwareMap, true);

        Action runSwiggly = drive.actionBuilder(startPos)
                .setTangent(0)
                .splineToLinearHeading(path1, Math.toRadians(0))
                .turnTo(Math.toRadians(-30))
                .splineToLinearHeading(path2, Math.toRadians(0))
                .turnTo(Math.toRadians(-30))
                .splineToLinearHeading(path3, Math.toRadians(0))
                .turnTo(Math.toRadians(-30))
                .splineToLinearHeading(path4, Math.toRadians(0))
                .turnTo(Math.toRadians(-30))
                .turnTo(Math.toRadians(0))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        getHeading(drive),
                        armSubsystem.controlActuators(),
                        new SequentialAction(
                                new ParallelAction(
                                        runSwiggly,
                                        swingArms(armSubsystem)
                                ),
                                terminate()
                        )
                )
        );
    }

    boolean finished = false;
    public Action getHeading(MecanumDrive drive) {
        return telemetryPacket -> {
            drive.updatePoseEstimate();
            telemetry.addData("Heading", drive.pose.heading);
            telemetry.update();
            return !finished;
        };
    }

    public Action terminate() {
        return telemetryPacket -> {
            finished = true;
            return false;
        };
    }

    public Action swingArms(ArmSubsystemAuto arm) {
        return new SequentialAction(
                new SleepAction(1),
                arm.moveV4B(ArmSubsystem.ArmState.LEFT),
                new SleepAction(1),
                arm.moveV4B(ArmSubsystem.ArmState.RIGHT),
                new SleepAction(1),
                arm.moveV4B(ArmSubsystem.ArmState.LEFT),
                new SleepAction(1),
                arm.moveV4B(ArmSubsystem.ArmState.RIGHT)
        );
    }
}
