package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemTesting;

//@Disabled
@TeleOp(name = "V4B Arm Testing", group = "Testing")
public class V4BTesting extends LinearOpMode {

    public void runOpMode(){
        ArmSubsystemTesting armSubsystem = new ArmSubsystemTesting(this);
        armSubsystem.initV4B(hardwareMap);

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
