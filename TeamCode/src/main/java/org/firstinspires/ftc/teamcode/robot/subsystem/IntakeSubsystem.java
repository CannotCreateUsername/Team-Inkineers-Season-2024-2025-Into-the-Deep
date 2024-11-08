package org.firstinspires.ftc.teamcode.robot.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeSubsystem {
    private final CRServo intake;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
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


    // Autonomous Actions

    ElapsedTime autoTimer = new ElapsedTime();

    public Action spinIntake() {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    autoTimer.reset();
                    set = true;
                }
                intake.setPower(1);
                return autoTimer.seconds() < 2;
            }
        };
    }
}
