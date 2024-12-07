package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemTeleOp;

//@Disabled
@TeleOp(name = "Arm Testing", group = "Testing")
public class ArmTest extends LinearOpMode {
    DcMotor armMotor = null;

    public void runOpMode(){
        ArmSubsystemTeleOp armSubsystem = new ArmSubsystemTeleOp();
        armSubsystem.init(hardwareMap, false);

        GamepadEx gam = new GamepadEx(gamepad1);

        waitForStart();
        while(opModeIsActive()) {
            armSubsystem.runV4BTesting(gam);

            gam.readButtons();

            telemetry.addData("Arm Telemetry", armSubsystem.armDisplayText);
            telemetry.update();
        }


    }
}
