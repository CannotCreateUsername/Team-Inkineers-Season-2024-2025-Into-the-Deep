package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/** @noinspection FieldCanBeLocal*/
public class ArmSubsystemTeleOp extends ArmSubsystem {

    public double driveMultiplier = 1;
    Pose2d drivePos;

    public void runSubsystem(GamepadEx gamepadEx1, GamepadEx gamepadEx2, Gamepad gamepad) {
        runSlides(gamepadEx1);
        runArm(gamepadEx1);
        runIntake(gamepad);
        runHang(gamepadEx1, gamepadEx2);
    }

    private int buttonCount = 0;
    ElapsedTime buttonTimer = new ElapsedTime();

    private double slidePower = DEFAULT_SLIDE_POWER;
    public void runSlides(GamepadEx gamepad) {
        // Arm control logic
        switch (slideState) {
            case REST:
                driveMultiplier = 1;
                // Prevent stalling
                if (stallTimer.seconds() > 2 || (slideSwitch.isPressed() && resetting)) {
                    resetSlideEncoders();
                }

                slideDisplayText = "REST";
                targetSlidePosition = REST_POSITION_SLIDES;

                if (gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    setSlideState(SlideState.INTAKE, false);

                    setArmState(ArmState.LEFT, false);
                    setWristState(WristState.DOWN, false);
                    setSpecimenState(SpecimenState.INTAKE);
                }
                break;
            case INTAKE:
                if (intakeState == IntakeState.IN) {
                    if (stallTimer.seconds() > 0.1)
                        targetSlidePosition = REST_POSITION_SLIDES;
                }
                else {
                    targetSlidePosition = INTAKE_POSITION_SLIDES;
                }
                setWristState(WristState.DOWN, false);

                if (gamepad.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
                    targetSlidePosition = REST_POSITION_SLIDES;
                    setSlideState(SlideState.REST, true);

                    setArmState(ArmState.REST, false);
                    setWristState(WristState.NEUTRAL, false);
                } else if (gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    targetSlidePosition = OUTTAKE_POSITION_SLIDES;
                    setSlideState(SlideState.OUTTAKE, false);

                    setArmState(ArmState.HANG, false);
                    setWristState(WristState.UP, false);
                }
                break;
            case OUTTAKE:
                driveMultiplier = 0.6;
                slideDisplayText = "OUTTAKE";

                if (gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    buttonCount++;
                }
                if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    targetSlidePosition -= MANUAL_INCREMENT;
                } else if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    targetSlidePosition += (MANUAL_INCREMENT);
                }

                // Double press to go back to REST
                if (buttonTimer.seconds() > 0.5) {
                    buttonTimer.reset();
                }
                if (buttonCount > 1 && buttonTimer.seconds() < 0.5) {
                    // Move back down to rest
                    setSlideState(SlideState.REST, true);
                    setArmState(ArmState.REST, false);
                    setWristState(WristState.NEUTRAL, true);

                    buttonCount = 0;
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
        switch (armState) {
            case REST:
                armDisplayText = "Rest";
                if (armTimer.seconds() > 0.5) {
                    setV4BPosition(ARM_REST_POS);
                }

                if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    setArmState(ArmState.LEFT, false);
                    setSpecimenState(SpecimenState.OUTTAKE);
                    setWristState(WristState.PICKUP, true);
                } else if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    setArmState(ArmState.INTAKE, true);
                    setSpecimenState(SpecimenState.INTAKE);
                    setWristState(WristState.PICKUP, true);
                }
                break;
            case LEFT:
                armDisplayText = "Left Extended";
                if (slideState == SlideState.INTAKE) {
                    setV4BPosition(V4B_LOWER_CENTER - 32.0/MAX_LOWER_BAR_ROTATION, V4B_UPPER_LEFT);
                } else {
                    setV4BPosition(ARM_LEFT_POS);
                }
                if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    setArmState(ArmState.REST, false);
                    setWristState(WristState.NEUTRAL, false);
                    setSpecimenState(SpecimenState.INTAKE);
                } else if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    setArmState(ArmState.INTAKE, false);
                    setWristState(WristState.PICKUP, true);
                    setSpecimenState(SpecimenState.INTAKE);
                }
                break;
            case INTAKE:
                armDisplayText = "Intake Extended";
                if (armTimer.seconds() > 0.4) {
                    setV4BPosition(ARM_INTAKE_POS);
                }
                if (gamepad.wasJustPressed(GamepadKeys.Button.X) || gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    setArmState(ArmState.REST, false);
                    setWristState(WristState.NEUTRAL, false);
                    setSpecimenState(SpecimenState.INTAKE);
                } else if (gamepad.isDown(GamepadKeys.Button.DPAD_LEFT) || gamepad.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
                    setArmState(ArmState.RIGHT, false);
                }
                break;
            case RIGHT:
                armDisplayText = "Intake Alternated";
                if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
                    setV4BPosition(V4B_LOWER_CENTER, 0.5 - UPPER_ALT_INTAKE_ANGLE);
                } else if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)) {
                    setV4BPosition(V4B_LOWER_CENTER, 0.5 + UPPER_ALT_INTAKE_ANGLE);
                } else if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    setArmState(ArmState.INTAKE, false);
                }
                break;
            case HANG:
                armDisplayText = "Hanging";
                setV4BPosition(V4B_LOWER_INITIAL, V4B_UPPER_INITIAL);
                break;
        }

        // The Specimen Arm should interact closely with the Virtual Four Bar.
        switch (specimenState) {
            case INTAKE:
                if (specimenTimer.seconds() > 0.6) {
                    specimenWrist.setPosition(SPECIMEN_WRIST_INTAKE_ANGLE);
                } else if (specimenTimer.seconds() > 0.4) {
                    specimenWrist.setPosition(SPECIMEN_WRIST_TRANSITION_OFF);
                }

                specimenBar.setPosition(SPECIMEN_BAR_INTAKE_ANGLE);

                if (gamepad.wasJustPressed(GamepadKeys.Button.B)) {
                    setArmState(ArmState.REST, false);
                    setWristState(WristState.UP, false);
                    setSpecimenState(SpecimenState.OUTTAKE);
                }
                break;
            case OUTTAKE:
                specimenWrist.setPosition(SPECIMEN_WRIST_OUTTAKE_ANGLE);
                specimenBar.setPosition(SPECIMEN_BAR_OUTTAKE_ANGLE);

                if (gamepad.wasJustPressed(GamepadKeys.Button.B)) {
                    setArmState(ArmState.REST, false);
                    setSpecimenState(SpecimenState.INTAKE);
                }
                break;
            case HANG:
                specimenBar.setPosition(SPECIMEN_BAR_OUTTAKE_ANGLE);
                specimenWrist.setPosition(SPECIMEN_WRIST_OUTTAKE_ANGLE);
                break;
        }

        switch (wristState) {
            case NEUTRAL:
                wristDisplayText = "Neutral";
                if (wristTimer.seconds() > 0.2)
                    intakeWrist.setPosition(WRIST_NEUTRAL);
                if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) {
                    wristState = WristState.PICKUP;
                }
                break;
            case UP:
                wristDisplayText = "Up";
                if (wristTimer.seconds() > 0.2)
                    intakeWrist.setPosition(WRIST_UP);
                if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) {
                    wristState = WristState.NEUTRAL;
                }
                break;
            case PICKUP:
                wristDisplayText = "Pickup";
                if (wristTimer.seconds() > 0.2)
                    intakeWrist.setPosition(WRIST_PICKUP);
                if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) {
                    wristState = WristState.LOW;
                }
                break;
            case LOW:
                wristDisplayText = "Low";
                if (wristTimer.seconds() > 0.1)
                    intakeWrist.setPosition(WRIST_LOW);
                if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) {
                    wristState = armState == ArmState.LEFT ? WristState.PICKUP : WristState.NEUTRAL;
                }
                break;
            case DOWN:
                wristDisplayText = "Down";
                if (wristTimer.seconds() > 0.1) {
                    intakeWrist.setPosition(WRIST_DOWN);
                }
                break;
        }
    }

    public void runIntake(Gamepad gamepad) {
        switch (intakeState) {
            case IDLE:
                intakeDisplayText = "IDLE";
                setIntakePowers(0);

                if (gamepad.left_trigger > 0) {
                    intakeState = IntakeState.OUT;
                    setWristState(WristState.PICKUP, false);
                } else if (gamepad.right_trigger > 0) {
                    intakeState = IntakeState.IN;
                    setWristState(slideState == SlideState.INTAKE ? WristState.DOWN : WristState.LOW, true);
                    stallTimer.reset();
                }
                break;
            case IN:
                intakeDisplayText = "IN";
                setIntakePowers(1);
                if (gamepad.right_trigger == 0) {
                    intakeState = IntakeState.IDLE;
                    setWristState(WristState.PICKUP, false);
                }
                break;
            case OUT:
                intakeDisplayText = "OuT";
                setIntakePowers(-0.4);
                if (gamepad.left_trigger == 0) {
                    intakeState = IntakeState.IDLE;
                }
                break;
            case HANG:
                break;
        }
    }

    ElapsedTime hangTimer = new ElapsedTime();
    private boolean unhang = false;
    public void runHang(GamepadEx gamepad1, GamepadEx gamepad2) {
        switch (hangState) {
            case REST:
                if (gamepad1.wasJustPressed(GamepadKeys.Button.BACK)) {
                    // tuck everything in
                    hangTimer.reset();
                    wormMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    wormMotor.setTargetPosition(0);
                    hangState = HangState.HANGING;
                }

                if (unhang) {
                    if (hangTimer.seconds() > 1.5) {
                        setArmState(ArmState.REST, false);
                        setSlideState(SlideState.REST, false);
                        setSpecimenState(SpecimenState.INTAKE);
                        unhang = false;
                    }
                    wormMotor.setTargetPosition(0);
                    wormMotor.setPower(0.8);
                    wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if (gamepad2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    wormMotor.setPower(-0.5);
                } else if (gamepad2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    wormMotor.setPower(0.5);
                } else {
                    wormMotor.setPower(0);
                }
                break;
            case HANGING:
                // Ensure arms are pulled backed
                setWristState(WristState.UP, false);
                setArmState(ArmState.HANG, false);
                setSlideState(SlideState.HANG, false);
                setSpecimenState(SpecimenState.HANG);

                if (hangTimer.seconds() > 3) {
                    hangDisplayText = "A: to Release Slides, X: to Grapple Worm";
                    wormMotor.setPower(0);
                    wormMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } else if (hangTimer.seconds() > 2) {
                    wormMotor.setTargetPosition(HANG_WORM_LV2);
                    targetSlidePosition = ASCENT_LV2;
                    wormMotor.setPower(0.8);
                } else if (hangTimer.seconds() > 1.5) {
                    targetSlidePosition = PRE_ASCENT_LV2;
                    wormMotor.setPower(0);
                } else if (hangTimer.seconds() > 0.5) {
                    wormMotor.setTargetPosition(HANG_WORM_READY);
                    wormMotor.setPower(0.8);
                } else {
                    targetSlidePosition = ASCENT_LV2_READY;
                    wormMotor.setPower(0);
                }

                // Manual Control
                if (gamepad1.isDown(GamepadKeys.Button.A)) {
                    targetSlidePosition += 30;
                } else if (gamepad1.isDown(GamepadKeys.Button.B)) {
                    targetSlidePosition -= 30;
                } else if (gamepad1.isDown(GamepadKeys.Button.X)) {
                    wormMotor.setPower(-0.8);
                } else if (gamepad1.isDown(GamepadKeys.Button.Y)) {
                    wormMotor.setPower(0.8);
                }

                if (gamepad1.wasJustPressed(GamepadKeys.Button.BACK)) {
                    hangTimer.reset();
                    wormMotor.setTargetPosition(0);
                    slidePower = DEFAULT_SLIDE_POWER;

                    unhang = true;
                    hangState = HangState.REST;
                }

                // Stop running to allow for manual control
                if (hangTimer.seconds() < 4) {
                    wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;
        }
    }
}
