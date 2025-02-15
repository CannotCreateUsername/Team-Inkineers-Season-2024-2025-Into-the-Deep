package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemTeleOp;

@TeleOp(name = "Proto Hang", group = "Testing")
public class TestHang extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ArmSubsystemTeleOp armSubsystem = new ArmSubsystemTeleOp(hardwareMap, true);

        GamepadEx gamepad = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            armSubsystem.runSlideMotorsPID(1);
            armSubsystem.runHang(gamepad, gamepadEx2);
            armSubsystem.specimenBar.setPosition(0.5);
            armSubsystem.specimenWrist.setPosition(0.5);
//            if (gamepad1.a) {
//                armSubsystem.hangMotor.setTargetPosition(500);
//            } else if (gamepad1.b) {
//                armSubsystem.hangMotor.setTargetPosition(1000);
//            } else if (gamepad1.x) {
//                armSubsystem.hangMotor.setTargetPosition(0);
//            }

//            armSubsystem.hangPID(1);

            gamepad.readButtons();
            telemetry.addData("Slide Position", armSubsystem.getSlidesPosition());
            telemetry.addData("Arm Position", armSubsystem.wormMotor.getCurrentPosition());
            telemetry.addData("Hang Target", armSubsystem.hangMotor.getTargetPosition());
            telemetry.addData("Hang Position", armSubsystem.hangMotor.getCurrentPosition());
            telemetry.addData("Hang Reset should be false to move", armSubsystem.isHangReset());
            telemetry.addData("Lower Limit Pressed", armSubsystem.lowerSwitch.isPressed());
            telemetry.addData("Upper Limit Pressed", armSubsystem.upperSwitch.isPressed());
            telemetry.update();
        }
    }
}
