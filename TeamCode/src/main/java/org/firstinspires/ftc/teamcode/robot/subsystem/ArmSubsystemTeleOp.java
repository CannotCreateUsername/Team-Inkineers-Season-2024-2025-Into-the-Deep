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
        hangMotor = hardwareMap.get(DcMotor.class, "hang_motor");
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
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize Positions; Start at REST
        armState = ArmState.REST;
        intakeState = IntakeState.IDLE;
        wristState = WristState.NEUTRAL;
        hangState = HangState.REST;

        targetSlidePosition = INTAKE_POSITION_SLIDES;
        wrist.setPosition(WRIST_UP);
    }

    ElapsedTime stallTimer = new ElapsedTime();
    public void runArm(GamepadEx gamepad) {
        // Arm control logic
        switch (armState) {
            case INTAKE:
                if (stallTimer.seconds() > 0.5) {
                    resetSlideEncoders();
                }
                slideDisplayText = "INTAKE";
                targetSlidePosition = REST_POSITION_SLIDES;
                break;
            case REST:
                slideDisplayText = "REST";
                targetSlidePosition = INTAKE_POSITION_SLIDES;
                if (gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    armState = ArmState.OUTTAKE;
                    targetSlidePosition = OUTTAKE_POSITION_SLIDES;
                    wristState = WristState.NEUTRAL;
                }
                break;
            case OUTTAKE:
                slideDisplayText = "OUTTAKE";
                if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    targetSlidePosition -= MANUAL_INCREMENT;
                } else if (gamepad.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
                    // Spin outtake and move back down to rest
                    armState = ArmState.REST;
                } else if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    targetSlidePosition += (MANUAL_INCREMENT);
                }
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

    public void runIntake(Gamepad gamepad, GamepadEx gamepadEx) {
        runWrist(gamepadEx);
        switch (intakeState) {
            case IDLE:
                intakeDisplayText = "IDLE";
                intake.setPower(0);
                intake2.setPower(0);

//                if (getInvalidColor() && intaked) {
//                    eject();
//                } else
                    if (gamepad.left_trigger > 0) {
                    intakeState = IntakeState.OUT;
                    wristState = WristState.NEUTRAL;
                } else if (gamepad.right_trigger > 0) {
                    if (wristState == WristState.DOWN) {
                        stallTimer.reset();
                        intakeState = IntakeState.IN;
                        armState = ArmState.INTAKE;
                    } else {
                        intakeState = IntakeState.IN;
                    }
                }
                break;
            case IN:
                intakeDisplayText = "IN";
                intake.setPower(1);
                intake2.setPower(1);

                if (gamepad.right_trigger == 0 ) { // || !getInvalidColor()
                    intakeState = IntakeState.IDLE;
                    if (wristState == WristState.DOWN)
                        armState = ArmState.REST;
                }
                break;
            case OUT:
                intakeDisplayText = "OuT";
                intake.setPower(-.4);
                intake2.setPower(-.4);

                if (gamepad.left_trigger == 0 && !eject) {
                    intakeState = IntakeState.IDLE;
                    wristState = WristState.NEUTRAL;
                } else {
                    if (eject && ejectTimer.seconds() > 0.8) {
                        intakeState = IntakeState.IDLE;
                        eject = false;
                    }
                }
                break;
        }
    }

    public void runWrist(GamepadEx gamepad) {
        // Max Rotation for 2000-0025-0002 Torque Servo: 300 degrees
        switch (wristState) {
            case NEUTRAL:
                wristDisplayText = "Neutral";
                wrist.setPosition(WRIST_NEUTRAL);
                if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    wristState = WristState.DOWN;
                }
                break;
            case UP:
                wristDisplayText = "Up";
                wrist.setPosition(WRIST_UP);
                break;
            case DOWN:
                wristDisplayText = "Down";
                wrist.setPosition(WRIST_DOWN);
                if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    wristState = WristState.NEUTRAL;
                }
                break;
        }
    }

    public void hangPID(double power) {
        double error = hangMotor.getTargetPosition() - hangMotor.getCurrentPosition();
        if (Math.abs(error) > 20) {
            hangMotor.setPower((error * 0.02)*power);
        } else {
            hangMotor.setPower(0.0);
        }
    }

    // Additional Hanging Code
    public void runHang(GamepadEx gamepad) {
        switch (hangState) {
            case REST:
                hangDisplayText = "REST";
                hangMotor.setTargetPosition(HANG_REST);
                hangServo.setPosition(0);

                if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
                    hangState = HangState.READY;
                }
                break;
            case READY:
                hangDisplayText = "READY";
                hangMotor.setTargetPosition(HANG_UP);
                hangServo.setPosition(.1);
                wristState = WristState.UP;

                if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
                    hangState = HangState.HANGING;
                } else if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    hangState = HangState.REST;
                    wristState = WristState.NEUTRAL;
                }
                break;
            case HANGING:
                hangDisplayText = "X to cancel";
                hangMotor.setTargetPosition(HANG_DOWN);
                wristState = WristState.UP;

                if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    hangState = HangState.READY;
                }
                break;
        }
        hangPID(1);
    }
}
