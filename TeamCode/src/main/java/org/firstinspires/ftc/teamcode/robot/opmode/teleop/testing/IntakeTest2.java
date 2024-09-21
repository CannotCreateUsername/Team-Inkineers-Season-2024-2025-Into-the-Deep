package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeTest2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo intake = hardwareMap.get(Servo.class, "intake");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");

        waitForStart();
        while (opModeIsActive()) {
            
        }
    }
}
