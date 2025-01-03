package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/** @noinspection FieldCanBeLocal*/
public class ArmSubsystemTeleOp extends ArmSubsystem {

    public double driveMultiplier = 1;
    Pose2d drivePos;

    public void runSubsystem(GamepadEx gamepadEx1, GamepadEx gamepadEx2, Gamepad gamepad) {
        runSlides(gamepadEx1);
        runArm(gamepadEx2);
        runIntake(gamepad);
        runHang();
    }

    private int buttonCount = 0;
    ElapsedTime stallTimer = new ElapsedTime();
    ElapsedTime buttonTimer = new ElapsedTime();
    public void runSlides(GamepadEx gamepad) {
        // Arm control logic
        switch (slideState) {
            case REST:
                driveMultiplier = 1;
                // Prevent stalling
                if (stallTimer.seconds() > 0.5 || slideSwitch.isPressed()) {
                    resetSlideEncoders();
                }

                slideDisplayText = "REST";
                targetSlidePosition = REST_POSITION_SLIDES;

                if (gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slideState = SlideState.OUTTAKE;
                    targetSlidePosition = OUTTAKE_POSITION_SLIDES;
                    wristState = WristState.SCORE;
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
                    buttonCount = 0;

                    // Move back down to rest
                    slideState = SlideState.REST;
                    stallTimer.reset();
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
        runSlideMotorsPID(DEFAULT_SLIDE_POWER);
    }

    public void getDrivePos(MecanumDrive drive) {
        drivePos = drive.pose;
    }

    private boolean toggleSide = false;
    public void runArm(GamepadEx gamepad) {
        switch (armState) {
            case REST:
                armDisplayText = "Rest";
                if (armTimer.seconds() > 0.5) {
                    setV4BPosition(ARM_REST_POS);
                }
                if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    if (toggleSide) {
                        setArmState(ArmState.INTAKE);
                        setSpecimenState(SpecimenState.OUTTAKE);
                        toggleSide = false;
                    } else {
                        setArmState(ArmState.LEFT);
                        setSpecimenState(SpecimenState.INTAKE);
                        toggleSide = true;
                    }
                    setWristState(WristState.NEUTRAL, true);
                    armTimer.reset();
                } else if (gamepad.wasJustPressed(GamepadKeys.Button.START)) {
                    setArmState(ArmState.MEGA_REST);
                }
                break;
            case LEFT:
                armDisplayText = "Left Extended";
                setV4BPosition(ARM_LEFT_POS);
                if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    setArmState(ArmState.REST);
                    setSpecimenState(SpecimenState.OUTTAKE);
                }
                break;
            case INTAKE:
                armDisplayText = "Right Extended";
                setV4BPosition(ARM_INTAKE_POS);
                if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    setArmState(ArmState.REST);
                }
                break;
            case MEGA_REST:
                armDisplayText = "Mega Rest";
                setV4BPosition(MEGA_REST_POS);
                if (gamepad.wasJustPressed(GamepadKeys.Button.START)) {
                    setArmState(ArmState.REST);
                }
                break;
            case HANG:
                armDisplayText = "Hanging";
                setV4BPosition(V4B_LOWER_RIGHT, V4B_UPPER_REST);
                break;
        }

        // The Specimen Arm should interact closely with the Virtual Four Bar.
        switch (specimenState) {
            case INTAKE:
                if (specimenTimer.seconds() > 0.2) {
                    specimenWrist.setPosition(SPECIMEN_WRIST_INTAKE_ANGLE);
                } else {
                    specimenWrist.setPosition(SPECIMEN_WRIST_TRANSITION_ANGLE);
                }
                specimenBar.setPosition(SPECIMEN_BAR_INTAKE_ANGLE);
                break;
            case OUTTAKE:
                specimenWrist.setPosition(SPECIMEN_WRIST_STRAIGHT_ANGLE);
                specimenBar.setPosition(SPECIMEN_BAR_STRAIGHT_ANGLE);
                break;
            case HANG:
                specimenBar.setPosition(SPECIMEN_BAR_NEUTRAL);
                specimenWrist.setPosition(SPECIMEN_WRIST_NEUTRAL);
                break;
        }

        switch (wristState) {
            case NEUTRAL:
                wristDisplayText = "Neutral";
                if (wristTimer.seconds() > 0.5)
                    intakeWrist.setPosition(WRIST_PICKUP);
                if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    wristState = WristState.LOW;
                }
                break;
            case UP: // Used in Hang
                wristDisplayText = "Up";
                if (wristTimer.seconds() > 0.5)
                    intakeWrist.setPosition(WRIST_UP);
                break;
            case LOW:
                wristDisplayText = "Low";
                if (wristTimer.seconds() > 0.5)
                    intakeWrist.setPosition(WRIST_PICKUP_LOW);
                if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    wristState = WristState.NEUTRAL;
                }
                break;
            case SCORE: // Unused
                wristDisplayText = "Scoring";
                if (wristTimer.seconds() > 0.5)
                    intakeWrist.setPosition(WRIST_SCORE);
                if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    wristState = WristState.NEUTRAL;
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
                } else if (gamepad.right_trigger > 0) {
                    intakeState = IntakeState.IN;
                }
                break;
            case IN:
                intakeDisplayText = "IN";
                setIntakePowers(1);
                if (gamepad.right_trigger == 0 ) {
                    intakeState = IntakeState.IDLE;
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

    public void runHang() {

    }
}
