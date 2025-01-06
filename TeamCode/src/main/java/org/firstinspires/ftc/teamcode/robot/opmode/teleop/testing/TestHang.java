package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemTeleOp;

@TeleOp(name = "Proto Hang", group = "Testing")
public class TestHang extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ArmSubsystemTeleOp armSubsystem = new ArmSubsystemTeleOp();
        armSubsystem.init(hardwareMap, true);

        GamepadEx gamepad = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            armSubsystem.runSlideMotorsPID(1);
            armSubsystem.runHang(gamepad);
            armSubsystem.specimenBar.setPosition(0.5);
            armSubsystem.specimenWrist.setPosition(0.5);

            gamepad.readButtons();
            telemetry.addData("Slide Position", armSubsystem.getSlidesPosition());
            telemetry.addData("Arm Position", armSubsystem.wormMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
