package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystem.meet0.IntakeSubsystem0;

interface RightSideAutoInterface {
    default void initialize(HardwareMap hardwareMap) {
        IntakeSubsystem0 intakeSubsystem = new IntakeSubsystem0(hardwareMap);
    }
}
