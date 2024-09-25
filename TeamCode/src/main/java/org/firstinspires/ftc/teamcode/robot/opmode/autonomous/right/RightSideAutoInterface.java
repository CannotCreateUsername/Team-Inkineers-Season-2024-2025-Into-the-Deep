package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystem.IntakeSubsystem;

interface RightSideAutoInterface {
    default void initialize(HardwareMap hardwareMap) {
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);
    }
}
