package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Disabled
@TeleOp(name = "Color Testing", group = "Testing")
public class ColorTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor racist = hardwareMap.get(ColorSensor.class, "racist");

        waitForStart();
        while (opModeIsActive()) {
            boolean specialIntaked = (racist.red() > 200) || (racist.blue() > 200);
            telemetry.addData("Red", racist.red());
            telemetry.addData("Blue", racist.blue());
            telemetry.addData("Green", racist.green());
            telemetry.addData("Intaked Specimen?", specialIntaked);
            telemetry.update();
        }
    }
}
