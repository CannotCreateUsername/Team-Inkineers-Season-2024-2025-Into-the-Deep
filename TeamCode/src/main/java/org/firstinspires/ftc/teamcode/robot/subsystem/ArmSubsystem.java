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
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

/** @noinspection FieldCanBeLocal*/
public class ArmSubsystem {
    private enum ArmState {
        REST,
        INTAKE,
        OUTTAKE,
    }

//    private enum SlideState {
//        RETRACT,
//        EXTEND
//    }

    ArmState armState;
//    SlideState slideState;

    private int targetArmPosition;
    public int targetSlidePosition;

    // Needs to be adjusted based on testing
    // Arm
    private final int REST_POSITION_ARM = 800;
    private final int INTAKE_POSITION = -260;
    private final int OUTTAKE_POSITION = 1800;

    // Linear Slides
    private final int REST_POSITION_SLIDES = 0;
    private final int EXTEND_POSITION = 1500;
    private final int MAX_EXTEND_POSITION = 3500;
    private final int MANUAL_INCREMENT = 20;

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
            m.setDirection(DcMotorSimple.Direction.REVERSE);
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        arm_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize Positions; Start at REST
        armState = ArmState.REST;
//        slideState = SlideState.RETRACT;

        targetArmPosition = REST_POSITION_ARM;
        targetSlidePosition = REST_POSITION_SLIDES;

//        arm_motor.setTargetPosition(targetArmPosition);

        // Reset Timers
//        buttonTimer1.reset();
//        limitTimer.reset();
        armTimer.reset();
    }

//    public void initializeArmPosition() {
//        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm_motor.setPower(0.5);
//    }

    // Method to run slide motors to position
    private void runSlideMotors(double power) {
        double error = targetSlidePosition - getSlidesPosition();
        for (DcMotorEx m : slideMotors) {
            if (Math.abs(error) > threshold_slides) {
                // setting negative power because MOTOR IS FREAKING BROKEN AHHHHHHHHHHHHHHHHHHHHHHHhhh
                m.setPower(-(error * kPslides)*power);
            } else {
                m.setPower(0.0);
            }
        }
    }

    // Method to run arm motor to position
    private void runArmMotor(double power) {
        arm_motor.setTargetPosition(targetArmPosition);
        arm_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm_motor.setPower(power);
    }

    public int getSlidesPosition() {
        return slideMotors.get(0).getCurrentPosition();
    }

//    ElapsedTime buttonTimer1 = new ElapsedTime();
//    ElapsedTime limitTimer = new ElapsedTime();
    ElapsedTime armTimer = new ElapsedTime();

//    int buttonCount1 = 0;
    public void run(GamepadEx gamepad) {
        // Arm control logic
        switch (armState) {
            case REST:
                // Run to positions
                armDisplayText = "Rest";
                slideDisplayText = "Retracted";
                targetSlidePosition = REST_POSITION_SLIDES;
                // Retract the Slides first before moving the arm
                if (armTimer.seconds() > 1.0) {
                    targetArmPosition = REST_POSITION_ARM;
                }

                // Controls
                if (gamepad.wasJustReleased(GamepadKeys.Button.A)) {
                    armState = ArmState.INTAKE;
                    targetArmPosition = INTAKE_POSITION;
                    targetSlidePosition = EXTEND_POSITION;
                } else if (gamepad.wasJustReleased(GamepadKeys.Button.B)) {
                    // Move arm to intake position if A is pressed
                    armState = ArmState.OUTTAKE;
                    targetArmPosition = OUTTAKE_POSITION;
                    targetSlidePosition = EXTEND_POSITION;
                }
                break;
            case INTAKE:
                // Run to positions
                armDisplayText = "Intaeking";

                // Controls
                if (gamepad.wasJustReleased(GamepadKeys.Button.A)) {
                    // Move arm to REST if A is pressed
                    armState = ArmState.REST;
                    armTimer.reset();
                } else if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    // Manual control down
                    targetSlidePosition -= MANUAL_INCREMENT;
                    slideDisplayText = "Going Back!";
                } else {
                    slideDisplayText = "Extended";
                }
                // Manual Worm Gear Control
                if (gamepad.isDown(GamepadKeys.Button.DPAD_UP)) {
                    targetArmPosition += MANUAL_INCREMENT/2;
                } else if (gamepad.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                    targetArmPosition -= MANUAL_INCREMENT/2;
                }
                break;
            case OUTTAKE:
                // Run to positions
                armDisplayText = "Outtakeing";

                // Controls
                if (gamepad.wasJustReleased(GamepadKeys.Button.B)) {
                    // Move arm to REST if A is pressed
                    armState = ArmState.REST;
                    armTimer.reset();
                } else if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    // Manual control down
                    targetSlidePosition -= MANUAL_INCREMENT;
                    slideDisplayText = "Going Down!";
                } else if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER) && armState == ArmState.OUTTAKE) {
                    // Manual control up
                    targetSlidePosition += MANUAL_INCREMENT;
                    slideDisplayText = "Going Up!";
                } else {
                    slideDisplayText = "Elevated";
                }
                break;
        }


        /*
        Combined Slide States with overall arm motion. 10/31/24
         */
//        // Linear slide control logic
//        switch (slideState) {
//            case RETRACT:
//                targetSlidePosition = REST_POSITION_SLIDES;
//                if (limitTimer.seconds() > 2.5) {
//                    for (DcMotorEx m : slideMotors) {
//                        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    }
//                }
//                if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
//                    slideDisplayText = "Elevated";
//                    targetSlidePosition = EXTEND_POSITION;
//                    slideState = SlideState.EXTEND;
//                }
//                break;
//            case EXTEND:
//                // Retract slides if left bumper is pressed
//                if (gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
//                    if (buttonTimer1.seconds() > 0.4) {
//                        buttonTimer1.reset();
//                        buttonCount1 = 0;
//                    }
//                    buttonCount1++;
//                } else if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER) && getSlidesPosition() > MANUAL_INCREMENT) {
//                    // Manual control down
//                    targetSlidePosition -= MANUAL_INCREMENT;
//                    slideDisplayText = "Going Down!";
//                } else if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER) && armState == ArmState.OUTTAKE) {
//                    // Manual control up
//                    targetSlidePosition += MANUAL_INCREMENT;
//                    slideDisplayText = "Going Up!";
//                } else {
//                    slideDisplayText = "Elevated";
//                }
//                // If left bumper double pressed, retract slides
//                if (buttonTimer1.seconds() < 0.4 && buttonCount1 > 1) {
//                    slideDisplayText = "Retracted";
//                    // Reset entire arm
//                    armState = ArmState.REST;
//                    slideState = SlideState.RETRACT;
//                    limitTimer.reset();
//                }
//                break;
//        }

        // Prevent linear slides from over-rotating
        if (targetSlidePosition > MAX_EXTEND_POSITION) {
            targetSlidePosition = MAX_EXTEND_POSITION;
        } else if (targetSlidePosition < 0) {
            targetSlidePosition = 0;
        }
        // Prevent worm gear from going past limits
        if (targetArmPosition < -500) {
            targetArmPosition = -400;
        }
        // Run methods to go to the target position
        runArmMotor(1.0);
        runSlideMotors(0.5);
    }

    public void initReset() {
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetTimer.reset();
    }
    ElapsedTime resetTimer = new ElapsedTime();
    public void runReset(GamepadEx gamepad, LinearOpMode opMode) {
        if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            arm_motor.setPower(-0.5);
        } else if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            arm_motor.setPower(0.5);
        } else {
            arm_motor.setPower(0);
        }
        if (gamepad.wasJustReleased(GamepadKeys.Button.A)) {
            resetTimer.reset();
            arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (!opMode.isStopRequested() && resetTimer.seconds() < 2.0) {
                arm_motor.setTargetPosition(REST_POSITION_ARM);
                arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_motor.setPower(0.5);
                opMode.telemetry.addLine("Initializing...");
                opMode.telemetry.update();
            }
            arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        opMode.telemetry.addLine("Reset!");
    }

    public String armDisplayText = "";
    public String slideDisplayText = "WEEWOOWEEWOOWEEWOOWEEWOOWEEEEEEEEEEEEEEEEEEEEEEE";
}
