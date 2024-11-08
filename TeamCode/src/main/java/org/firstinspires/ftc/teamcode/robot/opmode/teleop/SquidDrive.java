package org.firstinspires.ftc.teamcode.robot.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystem1;
import org.firstinspires.ftc.teamcode.robot.subsystem.GamepadHelper;
import org.firstinspires.ftc.teamcode.robot.subsystem.IntakeSubsystem;

@TeleOp(name = "Squid Drive", group = "Linear Opmode")
public class SquidDrive extends LinearOpMode {

    double leftXInput;
    double leftYInput;
    double rightXInput;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        ArmSubsystem1 armSubsystem = new ArmSubsystem1(hardwareMap);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);

        // More Gamepad functionality
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        GamepadHelper leftYHelper = new GamepadHelper();
        GamepadHelper leftXHelper = new GamepadHelper();
        GamepadHelper rightXHelper = new GamepadHelper();

        // Initialization Telemetry
        ElapsedTime initTimer = new ElapsedTime();
        while (!isStopRequested() && initTimer.seconds() < 2.0) {
            telemetry.addLine("Initializing...");
            telemetry.update();
        }
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            leftYInput = leftYHelper.getRampingValue(gamepad1.left_stick_y);
            leftXInput = leftXHelper.getRampingValue(gamepad1.left_stick_x);
            rightXInput = rightXHelper.getRampingValue(gamepad1.right_stick_x);

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(-leftYInput, -leftXInput), -rightXInput)
            );

            armSubsystem.run(gamepadEx1);
            intakeSubsystem.run(gamepad1);

            gamepadEx1.readButtons();
            telemetry.addData("Slide Telemetry", armSubsystem.slideDisplayText);
            telemetry.addData("Slide Position", armSubsystem.getSlidesPosition());
            telemetry.addData("Slide Target", armSubsystem.targetSlidePosition);
            telemetry.addData("Intake Telemetry", intakeSubsystem.getIntakeTelemetry());
            telemetry.update();
        }
    }
}
