package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

@Disabled
@TeleOp(name = "V4B Servo Position Adder", group = "Testing")
public class V4BServoPositionAdder extends LinearOpMode {

    List<Servo> lowerBar;
    Servo upperBar;

    @Override
    public void runOpMode() throws InterruptedException {

        // Same initialization as in ArmSubsystem.java
        //noinspection ArraysAsListWithZeroOrOneArgument
        lowerBar = Arrays.asList(
                hardwareMap.get(Servo.class, "lower_bar")
        );
        upperBar = hardwareMap.get(Servo.class, "upper_bar");

        // Reverse servo directions
        for (Servo s : lowerBar) {
            s.setDirection(Servo.Direction.REVERSE);
        }
        upperBar.setDirection(Servo.Direction.REVERSE);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                upperBar.setPosition(upperBar.getPosition() + 0.05);
            } else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
                upperBar.setPosition(upperBar.getPosition() - 0.05);
            }

            gamepadEx1.readButtons();
            telemetry.addData("Lower Bar Position", lowerBar.get(0).getPosition());
            telemetry.addData("Upper Bar Position", upperBar.getPosition());
            telemetry.update();
        }
    }
}
