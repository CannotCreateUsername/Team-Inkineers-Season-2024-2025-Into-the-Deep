package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Position Tester", group = "Testing")
public class PositionTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "worm_motor");
        DcMotor slides = hardwareMap.get(DcMotor.class, "slide1");

        slides.setDirection(DcMotor.Direction.REVERSE);

        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Arm Pos", armMotor.getCurrentPosition());
            telemetry.addData("Slides Pos", slides.getCurrentPosition());

            telemetry.update();
        }
    }
}
