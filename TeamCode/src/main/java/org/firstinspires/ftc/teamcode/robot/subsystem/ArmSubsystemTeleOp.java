package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/** @noinspection FieldCanBeLocal*/
public class ArmSubsystemTeleOp extends ArmSubsystem {

    public double driveMultiplier = 1;

    ElapsedTime stallTimer = new ElapsedTime();
    public void runSlides(GamepadEx gamepad) {
        // Arm control logic
        switch (slideState) {
            case INTAKE:
                driveMultiplier = 1;
                if (stallTimer.seconds() > 0.5) {
                    resetSlideEncoders();
                }
                slideDisplayText = "INTAKE";
                targetSlidePosition = REST_POSITION_SLIDES;
                break;
            case REST:
                driveMultiplier = 1;
                slideDisplayText = "REST";
                targetSlidePosition = INTAKE_POSITION_SLIDES;
                if (gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slideState = SlideState.OUTTAKE;
                    targetSlidePosition = OUTTAKE_POSITION_SLIDES;
                    wristState = WristState.SCORE;
                }
                break;
            case OUTTAKE:
                driveMultiplier = 0.6;
                slideDisplayText = "OUTTAKE";
                if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    targetSlidePosition -= MANUAL_INCREMENT;
                } else if (gamepad.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
                    // Spin outtake and move back down to rest
                    slideState = SlideState.REST;
                    wristState = WristState.UP;
                } else if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    targetSlidePosition += (MANUAL_INCREMENT);
                }
                break;
            case HANG:
                // Controls are moved to the Hanging control loop
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

    public void runArm(GamepadEx gamepad) {
        switch (armState) {
            case INTAKE_CLOSE:
                armDisplayText = "INTAKE CLOSE";
                setV4BPosition(LEFT_INTAKE_CLOSE);
                break;
            case INTAKE_FAR:
                armDisplayText = "INTAKE FAR";
                setV4BPosition(LEFT_INTAKE_FAR);
                break;
            case REST:
                armDisplayText = "REST";
                setV4BPosition(V4B_POSITION_REST);
                break;
        }
    }

    public void runIntake(Gamepad gamepad, GamepadEx gamepadEx) {
        runWrist(gamepadEx);
        switch (intakeState) {
            case IDLE:
                intakeDisplayText = "IDLE";
                setIntakePowers(0);

                if (gamepad.left_trigger > 0) {
                    intakeState = IntakeState.OUT;
                } else if (gamepad.right_trigger > 0) {
                    if (wristState == WristState.DOWN) {
                        stallTimer.reset();
                        intakeState = IntakeState.IN;
                        slideState = SlideState.INTAKE;
                    } else {
                        intakeState = IntakeState.IN;
                    }
                }
                break;
            case IN:
                intakeDisplayText = "IN";
                setIntakePowers(1);

                if (gamepad.right_trigger == 0 ) { // || !getInvalidColor()
                    intakeState = IntakeState.IDLE;
                    if (wristState == WristState.DOWN)
                        slideState = SlideState.REST;
                }
                break;
            case OUT:
                intakeDisplayText = "OuT";
                setIntakePowers(-0.4);

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
                break;
            case UP:
                wristDisplayText = "Up";
                wrist.setPosition(WRIST_UP);
                break;
            case DOWN:
                wristDisplayText = "Down";
                wrist.setPosition(WRIST_DOWN);
                break;
            case SCORE:
                wristDisplayText = "Scoring";
                wrist.setPosition(WRIST_SCORE);
        }
    }

    public void hangPID(double power) {
        double error = hangMotor.getTargetPosition() - hangMotor.getCurrentPosition();
        if (Math.abs(error) > 20) {
            hangMotor.setPower((error * 0.01)*power);
        } else {
            hangMotor.setPower(0.0);
        }
    }

    // Additional Hanging Code
    public void runHang(GamepadEx gamepad) {
        switch (hangState) {
            case REST:
                hangDisplayText = "REST";
                hangMotor.setTargetPosition(HANG_LINEAR_REST);
                hangServo.setPosition(0);

                if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
                    hangState = HangState.READY;
                }
                break;
            case READY:
                hangDisplayText = "X to cancel";
                hangMotor.setTargetPosition(HANG_LINEAR_UP);
                hangServo.setPosition(.5);
                wristState = WristState.UP;

                // DPAD Down to Ascend Level 2
                if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
                    // Run worm gear to ready
                    hangState = HangState.LEVEL2;
                    // Cancel
                } else if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    hangState = HangState.REST;
                    wristState = WristState.NEUTRAL;
                }
                break;
            case LEVEL2:
                hangDisplayText = "DPAD UP to Ascend";
                slideState = SlideState.HANG;
                wristState = WristState.UP;
                hangMotor.setTargetPosition(HANG_LINEAR_DOWN);

                // Manual adjust worm gear
                if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    wormMotor.setPower(0.8);
                } else if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    wormMotor.setPower(-0.8);
                } else {
                    wormMotor.setPower(0);
                }

                // DPAD Up to start Ascend Level 3
                if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    // Run slides to hang position once to allow for manual control later
                    targetSlidePosition = HANG_POSITION_SLIDES;
                    hangState = HangState.LEVEL3;
                }
                break;
            case LEVEL3:
                hangDisplayText = "DPAD Down to Suspend";
                // Manual adjust worm gear
                // Manual adjust worm gear
                if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    wormMotor.setPower(0.8);
                } else if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    wormMotor.setPower(-0.8);
                } else {
                    wormMotor.setPower(0);
                }
                // Move the slides up or down
                if (gamepad.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                    targetSlidePosition -= (MANUAL_INCREMENT+20);
                } else if (gamepad.isDown(GamepadKeys.Button.DPAD_UP)) {
                    targetSlidePosition += (MANUAL_INCREMENT+20);
                }
        }
        hangPID(1);
    }

    private void setSlidePowers(double power) {
        for (DcMotorEx m : slideMotors) {
            m.setPower(power);
        }
    }


    public void runSubsystem(GamepadEx gamepadEx, Gamepad gamepad) {
        runSlides(gamepadEx);
        runArm(gamepadEx);
        runIntake(gamepad, gamepadEx);
        runHang(gamepadEx);
    }

    public void runManualTesting(GamepadEx gamepad) {
        // Viper Slide Motion
        if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            setSlidePowers(1);
        } else if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            setSlidePowers(-1);
        } else {
            setSlidePowers(0);
        }

        // Worm Gear Motion
        if (gamepad.isDown(GamepadKeys.Button.Y)) {
            wormMotor.setPower(0.8);
        } else if (gamepad.isDown(GamepadKeys.Button.X)) {
            wormMotor.setPower(-0.8);
        } else {
            wormMotor.setPower(0);
        }

        // Hang Motor Motion
        if (gamepad.isDown(GamepadKeys.Button.DPAD_UP)) {
            hangMotor.setPower(1);
        } else if (gamepad.isDown(GamepadKeys.Button.DPAD_DOWN)) {
            hangMotor.setPower(-1);
        } else {
            hangMotor.setPower(0);
        }
    }
}
