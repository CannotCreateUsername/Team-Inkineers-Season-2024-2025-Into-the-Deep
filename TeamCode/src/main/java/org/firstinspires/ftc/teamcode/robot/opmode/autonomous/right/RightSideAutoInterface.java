package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystem.meet0.IntakeSubsystemZero;

interface RightSideAutoInterface {
    default void initialize(HardwareMap hardwareMap) {
        IntakeSubsystemZero intakeSubsystem = new IntakeSubsystemZero(hardwareMap);
    }
}
