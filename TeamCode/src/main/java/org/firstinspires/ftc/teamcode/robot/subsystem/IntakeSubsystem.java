package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {
    private final CRServo intake;
    private final Servo wrist;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
    }

    public void killItself() {
        intake.setPower(0);
        wrist.setPosition(0);
    }
}
