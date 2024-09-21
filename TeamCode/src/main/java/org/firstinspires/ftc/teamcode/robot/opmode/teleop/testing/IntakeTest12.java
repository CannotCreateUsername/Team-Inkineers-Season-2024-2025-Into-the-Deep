package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeTest12 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        Servo intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
