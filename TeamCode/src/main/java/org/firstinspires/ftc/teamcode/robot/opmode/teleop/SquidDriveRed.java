package org.firstinspires.ftc.teamcode.robot.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemTeleOp;
import org.firstinspires.ftc.teamcode.robot.subsystem.GamepadHelper;

@TeleOp(name = "Squid Drive Red", group = "Linear Opmode")
public class SquidDriveRed extends LinearOpMode {

    double leftXInput;
    double leftYInput;
    double rightXInput;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        ArmSubsystemTeleOp armSubsystem = new ArmSubsystemTeleOp(hardwareMap, true);

        // More Gamepad functionality
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        GamepadHelper leftYHelper = new GamepadHelper();
        GamepadHelper leftXHelper = new GamepadHelper();
        GamepadHelper rightXHelper = new GamepadHelper();

        // Initialization Telemetry
        ElapsedTime initTimer = new ElapsedTime();
        while (!isStopRequested() && initTimer.seconds() < 1.0) {
            telemetry.addLine("Initializing...");
            telemetry.update();
        }
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            leftYInput = leftYHelper.getRampingValue(gamepad1.left_stick_y) * armSubsystem.driveMultiplier;
            leftXInput = leftXHelper.getRampingValue(gamepad1.left_stick_x) * armSubsystem.driveMultiplier;
            rightXInput = rightXHelper.getRampingValue(gamepad1.right_stick_x) * armSubsystem.driveMultiplier * armSubsystem.driveMultiplier;

            Vector2d driveVector = new Vector2d(-leftYInput, -leftXInput);
            Rotation2d rotationAmount = drive.pose.heading.inverse().plus(Math.toRadians(90)); // Offset 90 Degrees due to new drive orientation
            Vector2d rotatedInput = rotationAmount.times(driveVector);

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            rotatedInput, -rightXInput)
            );
            drive.updatePoseEstimate();

            // Reset heading if started in wrong orientation
            telemetry.addData("Reset Heading", "Press Back");
            telemetry.addLine();
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.START)) {
                drive.pose = new Pose2d(0, 0, 0);
            }

            // Run the ARM
            armSubsystem.getDrivePos(drive);
            armSubsystem.runSubsystem(gamepadEx1, gamepadEx2, gamepad1);

            gamepadEx1.readButtons();
            gamepadEx2.readButtons();
            telemetry.addData("Slide Telemetry", armSubsystem.slideDisplayText);
            telemetry.addData("Slide Position", armSubsystem.getSlidesPosition());
            telemetry.addData("Slide Target", armSubsystem.targetSlidePosition);
            telemetry.addData("Wrist Telemetry", armSubsystem.wristDisplayText);
            telemetry.addData("Intake Telemetry", armSubsystem.intakeDisplayText);
            telemetry.addData("V4B Telemetry", armSubsystem.armDisplayText);
            telemetry.addData("Ascent Telemetry", armSubsystem.hangDisplayText);
            telemetry.addLine();
            telemetry.addData("Drive Heading", drive.pose.heading);
            telemetry.addData("Drive X", drive.pose.position.x);
            telemetry.addData("Drive Y", drive.pose.position.y);
            telemetry.addData("Color State", armSubsystem.getColorState());
            telemetry.update();
        }
    }
}
