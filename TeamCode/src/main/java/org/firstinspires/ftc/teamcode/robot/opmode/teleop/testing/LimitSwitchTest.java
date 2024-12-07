package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Magnetic Switch Test", group = "Testing")
public class LimitSwitchTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TouchSensor mLimSwitch = hardwareMap.touchSensor.get("slide_limit");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Slide Limit Switch", mLimSwitch.isPressed());
            telemetry.update();
        }
    }
}
