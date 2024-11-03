package org.firstinspires.ftc.teamcode.robot.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystem.IntakeSubsystem;

@TeleOp(name = "Squid Drive", group = "Linear Opmode")
public class SquidDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));
        ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);

        // More Gamepad functionality
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        ElapsedTime initTimer = new ElapsedTime();
        while (!isStopRequested() && initTimer.seconds() < 2.0) {
            telemetry.addLine("Initializing...");
            telemetry.update();
        }

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    0
                            ),
                            -gamepad1.right_stick_x*0.4
                    )
            );

            armSubsystem.run(gamepadEx1);
            intakeSubsystem.run(gamepad1);

            gamepadEx1.readButtons();
            telemetry.addData("Arm Telemetry", armSubsystem.armDisplayText);
            telemetry.addData("Slide Telemetry", armSubsystem.slideDisplayText);
            telemetry.addData("Slide Position", armSubsystem.getSlidesPosition());
            telemetry.addData("Slide Target", armSubsystem.targetSlidePosition);
            telemetry.addData("Intake Telemetry", intakeSubsystem.getIntakeTelemetry());
            telemetry.update();
        }
    }
}
