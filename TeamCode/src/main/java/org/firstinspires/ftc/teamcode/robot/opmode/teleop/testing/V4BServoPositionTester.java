package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

public class V4BServoPositionTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Same initialization as in ArmSubsystem.java
        List<Servo> lowerBar = Arrays.asList(
                hardwareMap.get(Servo.class, "lower_bar")
        );
        Servo upperBar = hardwareMap.get(Servo.class, "upper_bar");

        // Reverse servo directions
        for (Servo s : lowerBar) {
            s.setDirection(Servo.Direction.REVERSE);
        }
        upperBar.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Lower Bar Position", lowerBar.get(0).getPosition());
            telemetry.addData("Upper Bar Position", upperBar.getPosition());
            telemetry.update();
        }
    }
}
