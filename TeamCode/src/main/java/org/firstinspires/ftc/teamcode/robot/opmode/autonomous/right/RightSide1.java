package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Right Side 1", group = "Autonomous")
public class RightSide1 extends RightSideAuto {
    @Override
    void run() {
        intakeSubsystem.killItself();
        telemetry.addData("intake", "killed");
        telemetry.update();
    }
}
