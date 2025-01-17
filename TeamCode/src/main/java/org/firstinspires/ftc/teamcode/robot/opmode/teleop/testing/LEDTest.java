package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;

@Disabled
@TeleOp(name = "LED Test", group = "Testing")
public class LEDTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        LED red = hardwareMap.get(LED.class, "red");
        LED green = hardwareMap.get(LED.class, "green");

        GamepadEx gamepadE1 = new GamepadEx(gamepad1);

        red.off();
        green.off();

        waitForStart();

        boolean greenOn = false;
        boolean redOn = false;

        while (opModeIsActive()) {

            if (gamepadE1.wasJustPressed(GamepadKeys.Button.A)) {
                if (greenOn) {
                    green.off();
                } else {
                    green.on();
                }
                greenOn = !greenOn;
            } else if (gamepadE1.wasJustPressed(GamepadKeys.Button.B)) {
                if (redOn) {
                    red.off();
                } else {
                    red.on();
                }
                redOn = !redOn;
            }

            gamepadE1.readButtons();
            telemetry.addData("B", "Toggle RED");
            telemetry.addData("A", "Toggle GREEN");
            telemetry.update();
        }
    }
}
