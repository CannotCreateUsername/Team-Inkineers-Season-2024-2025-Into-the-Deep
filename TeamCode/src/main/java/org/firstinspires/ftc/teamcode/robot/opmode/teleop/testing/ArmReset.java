package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystem.meet0.ArmSubsystemZero;

@Disabled
@TeleOp(name = "Arm Reset", group = "Testing")
public class ArmReset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ArmSubsystemZero armSubsystem = new ArmSubsystemZero(hardwareMap);
        armSubsystem.initReset();
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        waitForStart();
        while (opModeIsActive()) {
            armSubsystem.runReset(gamepadEx, this);
            gamepadEx.readButtons();
            telemetry.addData("Move Arm", "Right/Left Bumpers");
            telemetry.addData("Initialize Position", "Button A");
            telemetry.update();
        }
    }
}
