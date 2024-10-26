package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

/** @noinspection FieldCanBeLocal*/
public class ArmSubsystem {
    private enum ArmState {
        REST,
        INTAKE,
        OUTTAKE
    }
    private enum SlideState {
        REST,
        EXTEND
    }
    ArmState armState;
    SlideState slideState;

    private int targetArmPosition;
    private int targetSlidePosition;

    // Needs to be adjusted based on testing
    private final int REST_POSITION_ARM = 100;
    private final int INTAKE_POSITION = 0;
    private final int OUTTAKE_POSITION = 1000;

    private final int REST_POSITION_SLIDES = 0;
    private final int EXTEND_POSITION = 1000;

    private final DcMotorEx arm_motor;
    private final List<DcMotorEx> slideMotors; // Initialize as list to support potential multiple motors

    public ArmSubsystem(HardwareMap hardwareMap) {
        // Map Motors
        arm_motor = hardwareMap.get(DcMotorEx.class, "worm_motor");
        //noinspection ArraysAsListWithZeroOrOneArgument
        slideMotors = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, "slide1")
//                hardwareMap.get(DcMotorEx.class, "slide2")
        );

        // Set Motor Modes
        for (DcMotorEx m : slideMotors) {
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        // Initialize Positions; Start at REST
        armState = ArmState.REST;
        slideState = SlideState.REST;

        targetArmPosition = REST_POSITION_ARM;
        targetSlidePosition = REST_POSITION_SLIDES;

        // Reset Timers
        buttonTimer1.reset();
    }

    // Method to run slide motors to position
    private void runSlideMotors(int position) {
        for (DcMotorEx m : slideMotors) {
            m.setTargetPosition(position);
            m.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            m.setPower(1.0);
        }
    }

    // Method to run arm motor to position
    private void runArmMotor(int position) {
        arm_motor.setTargetPosition(position);
        arm_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm_motor.setPower(1.0);
    }

    public int getSlidesPosition() {
        return slideMotors.get(0).getCurrentPosition();
    }

    ElapsedTime buttonTimer1 = new ElapsedTime();
    int buttonCount1 = 0;
    public void run(GamepadEx gamepad) {
        // Arm control logic
        switch (armState) {
            case REST:
                armDisplayText = "Ready";
                targetArmPosition = REST_POSITION_ARM;
                if (gamepad.wasJustReleased(GamepadKeys.Button.A)) {
                    // Move arm to intake position if A is pressed
                    armState = ArmState.INTAKE;
                }
                break;
            case INTAKE:
                armDisplayText = "Intaeking";
                targetArmPosition = INTAKE_POSITION;
                if (gamepad.wasJustReleased(GamepadKeys.Button.A)) {
                    // Move arm to outtake position if A is pressed
                    armState = ArmState.OUTTAKE;
                } else if (gamepad.wasJustPressed(GamepadKeys.Button.B)) {
                    // Move arm to rest position if B is pressed
                    armState = ArmState.REST;
                }
                break;
            case OUTTAKE:
                armDisplayText = "Outtakeing";
                targetArmPosition = OUTTAKE_POSITION;
                if (gamepad.wasJustReleased(GamepadKeys.Button.A)) {
                    // Move arm to intake position if A is pressed
                    armState = ArmState.INTAKE;
                } else if (gamepad.wasJustPressed(GamepadKeys.Button.B)) {
                    // Move arm to rest position if B is pressed
                    armState = ArmState.REST;
                }
                break;
        }

        // Linear slide control logic
        switch (slideState) {
            case REST:
                if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slideDisplayText = "Elevated";
                    targetSlidePosition = EXTEND_POSITION;
                    slideState = SlideState.EXTEND;
                }
                break;
            case EXTEND:
                // Retract slides if left bumper is pressed
                if (gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    if (buttonTimer1.seconds() < 0.4) {
                        buttonCount1++;
                    } else {
                        buttonCount1 = 0;
                        buttonCount1++;
                        buttonTimer1.reset();
                    }
                    targetSlidePosition = REST_POSITION_SLIDES;
                } else if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER) && getSlidesPosition() > 10) {
                    // Manual control down
                    targetSlidePosition = getSlidesPosition() - 100;
                    slideDisplayText = "Going Down!";
                } else if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER) && getSlidesPosition() < 2000) {
                    // Manual control up
                    targetSlidePosition = getSlidesPosition() + 100;
                    slideDisplayText = "Going Up!";
                } else {
                    slideDisplayText = "Elevated";
                }
                // If left bumper double pressed, retract slides
                if (buttonTimer1.seconds() < 0.4 && buttonCount1 > 1) {
                    slideDisplayText = "Retracted";
                    slideState = SlideState.REST;
                }
                break;
        }

        // Run methods to go to the target position
        runArmMotor(targetArmPosition);
        runSlideMotors(targetSlidePosition);
    }

    public String armDisplayText = "";
    public String slideDisplayText = "WEEWOOWEEWOOWEEWOOWEEWOOWEEEEEEEEEEEEEEEEEEEEEEE";
}
