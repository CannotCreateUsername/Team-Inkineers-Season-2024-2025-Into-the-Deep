package org.firstinspires.ftc.teamcode.robot.subsystem;

import static org.firstinspires.ftc.teamcode.robot.constants.PIDConstants.kDhang;
import static org.firstinspires.ftc.teamcode.robot.constants.PIDConstants.kPhang;
import static org.firstinspires.ftc.teamcode.robot.constants.PIDConstants.threshold_hang;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/** @noinspection FieldCanBeLocal*/
public class ArmSubsystemTeleOp extends ArmSubsystem {

    public double driveMultiplier = 1;
    Pose2d drivePos = new Pose2d(0, 0, 0);

    ElapsedTime stallTimer = new ElapsedTime();

    private double slidePower = DEFAULT_SLIDE_POWER;
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
        runSlideMotorsPID(slidePower);
    }

    public void getDrivePos(MecanumDrive drive) {
        drivePos = drive.pose;
    }

    public void runArm(GamepadEx gamepad) {
        switch (areaState) {
            case CLOSE:
                if (drivePos.position.y > 50) {
                    wristState = WristState.DOWN;
                    areaState = AreaState.FAR;
                }
                break;
            case FAR:
                if (drivePos.position.y < 50) {
                    wristState = WristState.NEUTRAL;
                    areaState = AreaState.CLOSE;
                }
                break;
        }

        switch (armState) {
            case REST:
                armDisplayText = "Rest";
                setV4BPosition(ARM_REST_POS);
                if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    armState = ArmState.LEFT_FAR;
                } else if (gamepad.wasJustPressed(GamepadKeys.Button.B)) {
                    armState = ArmState.RIGHT_FAR;
                }
                break;
            case LEFT_FAR:
                armDisplayText = "Left Extended";
                setV4BPosition(ARM_LEFT_POS);
                if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) {
                    armState = ArmState.REST;
                } else if (gamepad.wasJustPressed(GamepadKeys.Button.B)) {
                    armState = ArmState.RIGHT_FAR;
                }
                break;
            case RIGHT_FAR:
                armDisplayText = "Right Extended";
                setV4BPosition(ARM_RIGHT_POS);
                if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) {
                    armState = ArmState.REST;
                } if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    armState = ArmState.LEFT_FAR;
                }
                break;
        }


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
                if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    wristState = WristState.NEUTRAL;
                }
                break;
            case DOWN:
                wristDisplayText = "Down";
                wrist.setPosition(WRIST_DOWN);
                if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    wristState = WristState.NEUTRAL;
                }
                break;
            case SCORE:
                wristDisplayText = "Scoring";
                wrist.setPosition(WRIST_SCORE);
        }


    }

    public void runIntake(Gamepad gamepad, GamepadEx gamepadEx) {
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

                if (gamepad.right_trigger == 0 ) {
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
            case HANG:
        }
    }

    public void hangPID(double power) {
        double error = hangMotor.getTargetPosition() - hangMotor.getCurrentPosition();
        if (Math.abs(error) > threshold_hang) {
            hangMotor.setPower((error * kPhang + kDhang)*power);
        } else {
            hangMotor.setPower(0.0);
        }
    }

    // Additional Hanging Code
    public void runHang(GamepadEx gamepad) {
        switch (hangState) {
            case REST:
                // Arm subsystem normal functionality, linear actuator down.
                hangDisplayText = "REST";
                hangMotor.setTargetPosition(HANG_LINEAR_REST);
                latchServo.setPosition(0);

                if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
                    hangState = HangState.READY;
                    intakeState = IntakeState.HANG; // Prevent any outside control from intake loop
                    stallTimer.reset(); // Prevent stalling when moving slides to REST
                }
                break;
            case READY:
                hangDisplayText = "X to cancel";
                hangMotor.setTargetPosition(HANG_LINEAR_UP);
                latchServo.setPosition(.5); // Unlatch worm gear
                slideState = SlideState.INTAKE;
                wristState = WristState.UP;

                // DPAD Down to Ascend Level 2
                if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
                    slideState = SlideState.HANG; // Prevent any outside control from slide loop
                    slidePower = 1; // More power to hang.
                    targetSlidePosition = HANG_POSITION_SLIDES;
                    hangState = HangState.LEVEL2;
                } else if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    // Back to default loops
                    hangState = HangState.REST;
                    slideState = SlideState.REST;
                    intakeState = IntakeState.IDLE;
                    wristState = WristState.NEUTRAL;
                }
                break;
            case LEVEL2:
                hangDisplayText = "DPAD UP to Ascend";
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
                if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    targetSlidePosition = HANG_POSITION_SLIDES - 1500;
                    hangMotor.setTargetPosition(HANG_LINEAR_REST);
                    hangState = HangState.LEVEL3;
                } else if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    hangState = HangState.READY;
                    slidePower = 0.5;
                }
                break;
            case LEVEL3:
                hangDisplayText = "Ascended!!!";

                // Move the linear actuator up or down
                if (gamepad.isDown(GamepadKeys.Button.DPAD_UP)) {
                    hangMotor.setTargetPosition(hangMotor.getCurrentPosition() + MANUAL_INCREMENT);
                } else if (gamepad.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                    hangMotor.setTargetPosition(hangMotor.getCurrentPosition() - MANUAL_INCREMENT);
                }

                // Move the slides up or down
                if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    targetSlidePosition -= (MANUAL_INCREMENT+20);
                } else if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
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
