package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

/** @noinspection FieldCanBeLocal*/
public class ArmSubsystemTeleOp extends ArmSubsystem {

    @Override
    public void init(HardwareMap hardwareMap, boolean isRedAlliance) {
        // For color sensor
        redSide = isRedAlliance;

        // Map the actuators
        slideMotors = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, "belt_slide"),
                hardwareMap.get(DcMotorEx.class, "non_slide")
        );
        intake = hardwareMap.get(CRServo.class, "right");
        intake2 = hardwareMap.get(CRServo.class, "left");
        wrist = hardwareMap.get(Servo.class, "wrist");
        hangMotor = hardwareMap.get(DcMotorEx.class, "hang_motor");
        hangServo = hardwareMap.get(Servo.class, "hang_servo");
        racist = hardwareMap.get(ColorSensor.class, "racist");

        // Set Motor Modes & Directions
        for (DcMotorEx m : slideMotors) {
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        // Reverse Encoder Motor
        slideMotors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);

        hangMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize Positions; Start at REST
        armState = ArmState.REST;
        intakeState = IntakeState.IDLE;
        wristState = WristState.UP;
        hangState = HangState.REST;

        targetSlidePosition = REST_POSITION_SLIDES;
        wrist.setPosition(WRIST_UP);
    }

    ElapsedTime stallTimer = new ElapsedTime();
    public void runArm(GamepadEx gamepad) {
        // Arm control logic
        switch (armState) {
            case REST:
                if (stallTimer.seconds() > 1.5) {
                    resetSlideEncoders();
                }
                slideDisplayText = "REST";
                targetSlidePosition = REST_POSITION_SLIDES;
                if (gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    armState = ArmState.INTAKE;
                }
                break;
            case INTAKE:
                slideDisplayText = "INTAKE";
                targetSlidePosition = INTAKE_POSITION_SLIDES;
                if (gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    armState = ArmState.OUTTAKE;
                    targetSlidePosition = OUTTAKE_POSITION_SLIDES;
                }
                break;
            case OUTTAKE:
                slideDisplayText = "OUTTAKE";
                if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    targetSlidePosition -= MANUAL_INCREMENT;
                } else if (gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    // Spin outtake and move back down to rest
                    eject();
                    stallTimer.reset();
                    armState = ArmState.REST;
                }
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
        runSlideMotorsPID(0.5);
    }

    private boolean intaked = false;
    public void runIntake(Gamepad gamepad) {
        switch (intakeState) {
            case IDLE:
                intake.setPower(0);
                intake2.setPower(0);
                intakeDisplayText = "IDLE";

                // Default Wrist States
                if (armState == ArmState.REST) {
                    wristState = WristState.UP;
                } else {
                    wristState = WristState.NEUTRAL;
                }

                if (!getValidColor() && intaked) {
                    eject();
                } else if (gamepad.left_trigger > 0) {
                    intakeState = IntakeState.OUT;
                    wristState = WristState.NEUTRAL;
                } else if (gamepad.right_trigger > 0) {
                    intakeState = IntakeState.IN;
                    if (armState == ArmState.REST) {
                        wristState = WristState.DOWN;
                    }
                }
                break;
            case IN:
                intake.setPower(1);
                intake2.setPower(1);
                intakeDisplayText = "IN";

                if (gamepad.right_trigger == 0) {
                    intakeState = IntakeState.IDLE;
                    intaked = true;
                }
                break;
            case OUT:
                intake.setPower(-1);
                intake2.setPower(-1);
                intakeDisplayText = "OuT";

                if (gamepad.left_trigger == 0 && !eject) {
                    intakeState = IntakeState.IDLE;
                    intaked = false;
                } else {
                    if (eject && ejectTimer.seconds() > 0.8) {
                        intakeState = IntakeState.IDLE;
                        intaked = false;
                        eject = false;
                    }
                }
                break;
        }
        runWrist();
    }

    public void runWrist() {
        // Max Rotation for 2000-0025-0002 Torque Servo: 300 degrees
        switch (wristState) {
            case NEUTRAL:
                wristDisplayText = "Neutral";
                wrist.setPosition(WRIST_NEUTRAL);
                break;
            case UP:
                wristDisplayText = "Up";
                wrist.setPosition(WRIST_UP);
                break;
            case DOWN:
                wristDisplayText = "Down";
                wrist.setPosition(WRIST_DOWN);
                break;
        }
    }

    // Additional Hanging Code
    public void runHang(GamepadEx gamepad) {
        switch (hangState) {
            case REST:
                hangMotor.setTargetPosition(HANG_REST);
                hangServo.setPosition(0);

                if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
                    hangState = HangState.READY;
                }
                break;
            case READY:
                hangMotor.setTargetPosition(HANG_UP);
                hangServo.setPosition(.1);

                if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
                    hangState = HangState.HANGING;
                } else if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    hangState = HangState.REST;
                }
                break;
            case HANGING:
                hangMotor.setTargetPosition(HANG_DOWN);

                if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    hangState = HangState.REST;
                }
                break;
        }
        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
