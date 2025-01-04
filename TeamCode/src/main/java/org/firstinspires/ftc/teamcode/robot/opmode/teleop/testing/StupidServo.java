package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Singular Test", group = "Testing")
public class StupidServo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo lowerBar = hardwareMap.get(Servo.class, "lower_bar");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                lowerBar.setPosition(1);
            } else if (gamepad1.b) {
                lowerBar.setPosition(0);
            } else if (gamepad1.start) {
                lowerBar.setPosition(0.5);
            }

            telemetry.addData("Go", "Axo n stupid sthi");
            telemetry.update();
        }
    }
}
