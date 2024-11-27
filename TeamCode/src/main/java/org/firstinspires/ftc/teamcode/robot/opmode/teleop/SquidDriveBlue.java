package org.firstinspires.ftc.teamcode.robot.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemTeleOp;
import org.firstinspires.ftc.teamcode.robot.subsystem.GamepadHelper;

@TeleOp(name = "Squid Drive Blue", group = "Linear Opmode")
public class SquidDriveBlue extends LinearOpMode {

    double leftXInput;
    double leftYInput;
    double rightXInput;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        ArmSubsystemTeleOp armSubsystem = new ArmSubsystemTeleOp();
        armSubsystem.init(hardwareMap, false);

        CRServo windmill = hardwareMap.get(CRServo.class, "windmill");

        // More Gamepad functionality
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
//        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

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
            rightXInput = rightXHelper.getRampingValue(gamepad1.right_stick_x) * armSubsystem.driveMultiplier;

            Vector2d driveVector = new Vector2d(-leftYInput, -leftXInput);
            Rotation2d rotationAmount = drive.pose.heading.inverse().plus(0);
            Vector2d rotatedInput = rotationAmount.times(driveVector);

            drive.setDrivePowers(
                    new PoseVelocity2d(rotatedInput
                            , -rightXInput)
            );
            drive.updatePoseEstimate();

            // Reset heading if started in wrong orientation
            telemetry.addData("Reset Heading", "Press Back");
            telemetry.addLine();
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.BACK)) {
                drive.pose = new Pose2d(0, 0, 0);
            }

            // Run the ARM
            armSubsystem.runSubsystem(gamepadEx1, gamepad1);
            // Troll
            windmill.setPower(rotatedInput.x+rightXInput);

            gamepadEx1.readButtons();
            telemetry.addData("Slide Telemetry", armSubsystem.slideDisplayText);
            telemetry.addData("Slide Position", armSubsystem.getSlidesPosition());
            telemetry.addData("Slide Target", armSubsystem.targetSlidePosition);
            telemetry.addData("Wrist Telemetry", armSubsystem.wristDisplayText);
            telemetry.addData("Intake Telemetry", armSubsystem.intakeDisplayText);
            telemetry.addData("Ascent Telemetry", armSubsystem.hangDisplayText);
            telemetry.addData("Hang Position:", armSubsystem.hangMotor.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Drive Heading", drive.pose.heading);
            telemetry.update();
        }
    }
}
