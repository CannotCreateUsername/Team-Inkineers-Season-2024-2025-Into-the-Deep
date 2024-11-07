package org.firstinspires.ftc.teamcode.robot.subsystem;

import static org.firstinspires.ftc.teamcode.robot.constants.PIDConstants.kPslides;
import static org.firstinspires.ftc.teamcode.robot.constants.PIDConstants.threshold_slides;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

/** @noinspection FieldCanBeLocal*/
public class ArmSubsystem1 {
    private enum ArmState {
        REST,
        INTAKE,
        OUTTAKE,
        HANG
    }

    ArmState armState;

    public int targetSlidePosition;

    // Needs to be adjusted based on testing
    // Linear Slides
    private final int REST_POSITION_SLIDES = 0;
    private final int MAX_EXTEND_POSITION = 3500;
    private final int MANUAL_INCREMENT = 20;

    private final List<DcMotorEx> slideMotors; // Initialize as list to support potential multiple motors
    private final Servo wrist;
    private final DcMotorEx hangMotor;
    private final Servo hangServo;

    public ArmSubsystem1(HardwareMap hardwareMap) {
        // Map the actuators
        slideMotors = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, "slide1"),
                hardwareMap.get(DcMotorEx.class, "slide2")
        );
        wrist = hardwareMap.get(Servo.class, "wrist");
        hangMotor = hardwareMap.get(DcMotorEx.class, "hang_motor");
        hangServo = hardwareMap.get(Servo.class, "hang_servo");

        // Set Motor Modes
        for (DcMotorEx m : slideMotors) {
            m.setDirection(DcMotorSimple.Direction.REVERSE);
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        hangMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize Positions; Start at REST
        armState = ArmState.REST;

        targetSlidePosition = REST_POSITION_SLIDES;

        armTimer.reset();
    }

    // Method to run slide motors to position
    private void runSlideMotors(double power) {
        double error = targetSlidePosition - getSlidesPosition();
        for (DcMotorEx m : slideMotors) {
            if (Math.abs(error) > threshold_slides) {
                // setting negative power because MOTOR IS FREAKING BROKEN AHHHHHHHHHHHHHHHHHHHHHHHhhh
                m.setPower((error * kPslides)*power);
            } else {
                m.setPower(0.0);
            }
        }
    }

    public int getSlidesPosition() {
        return slideMotors.get(0).getCurrentPosition();
    }

    ElapsedTime armTimer = new ElapsedTime();

    public void run(GamepadEx gamepad) {
        // Arm control logic
        switch (armState) {
            case REST:

                break;
            case INTAKE:

                break;
            case OUTTAKE:

                break;
            case HANG:
                break;
        }

        // Prevent linear slides from over-rotating
        if (targetSlidePosition > MAX_EXTEND_POSITION) {
            targetSlidePosition = MAX_EXTEND_POSITION;
        } else if (targetSlidePosition < 0) {
            targetSlidePosition = 0;
        }

        // Run methods to go to the target position
        runSlideMotors(0.5);
    }

    public String slideDisplayText = "WEEWOOWEEWOOWEEWOOWEEWOOWEEEEEEEEEEEEEEEEEEEEEEE";
}
