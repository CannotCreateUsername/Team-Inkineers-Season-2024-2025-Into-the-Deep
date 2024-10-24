package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {
    private final CRServo intake;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(CRServo.class, "intake");
    }

    public void killItself() {
        intake.setPower(0);
    }

    public void run(Gamepad gamepad) {
        // Use triggers to control intake power/direction
        intake.setPower(gamepad.right_trigger - gamepad.left_trigger);
    }

    // Returns the current direction of the intake
    public String getIntakeTelemetry() {
        if (intake.getPower() > 0) {
            return "IN";
        } else if (intake.getPower() < 0) {
            return "OUT";
        } else {
            return "IDLE";
        }
    }
}
