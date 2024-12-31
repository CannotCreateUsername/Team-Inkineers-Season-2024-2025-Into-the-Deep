package org.firstinspires.ftc.teamcode.robot.subsystem.meet2;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/** @noinspection FieldCanBeLocal*/
public class ArmSubsystemTeleOp2 extends ArmSubsystem2 {

    public double driveMultiplier = 1;
    Pose2d drivePos = new Pose2d(0, 0, 0);

    ElapsedTime stallTimer = new ElapsedTime();

    private double slidePower = DEFAULT_SLIDE_POWER;
    public void runSlides(GamepadEx gamepad) {
        // Arm control logic
        switch (slideState) {
            case INTAKE:
                driveMultiplier = 1;
                if (stallTimer.seconds() > 0.5 || slideSwitch.isPressed()) {
                    resetSlideEncoders();
                }
                // Let intake move first.
                if (stallTimer.seconds() > 0.15) {
                    targetSlidePosition = REST_POSITION_SLIDES;
                }
                slideDisplayText = "INTAKE";
                break;
            case REST:
                driveMultiplier = 1;
                slideDisplayText = "REST";
                if (wristState == WristState.DOWN) {
                    targetSlidePosition = INTAKE_SAMPLE_POSITION_SLIDES;
                } else if (wristState == WristState.LOW) {
                    targetSlidePosition = INTAKE_SAMPLE_POSITION_SLIDES - 100;
                } else {
                    if (intaked) {
                        targetSlidePosition = INTAKE_SPECIMEN_POSITION_SLIDES + 100;
                    } else {
                        targetSlidePosition = INTAKE_SPECIMEN_POSITION_SLIDES;
                    }
                }
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

    private boolean toggleSide = false;
    public void runArm(GamepadEx gamepad) {
        switch (armState) {
            //  Y < (driver front) intake
            // X B < (driver left, right) wrist, rest
            //  A < (driver right)
            case REST:
                armDisplayText = "Rest";
                if (armTimer.seconds() > 0.5) {
                    setV4BPosition(ARM_REST_POS);
                }
                if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    if (toggleSide) {
                        armState = ArmState.RIGHT_FAR;
                        setSpecimenState(SpecimenState.OUTTAKE);
                        toggleSide = false;
                    } else {
                        armState = ArmState.LEFT_FAR;
                        setSpecimenState(SpecimenState.INTAKE);
                        toggleSide = true;
                    }
                    setWristState(WristState.NEUTRAL, true);
                    armTimer.reset();
                } else if (gamepad.wasJustPressed(GamepadKeys.Button.START)) {
                    armState = ArmState.MEGA_REST;
                }
                break;
            case LEFT_FAR:
                armDisplayText = "Left Extended";
                if (armTimer.seconds() < 0.3) {
                    setV4BPosition(V4B_LOWER_CENTER, V4B_UPPER_TRANSITION);
                } else {
                    setV4BPosition(ARM_LEFT_POS);
                }
                if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    resetV4B();
                    setSpecimenState(SpecimenState.OUTTAKE);
                }
                break;
            case RIGHT_FAR:
                armDisplayText = "Right Extended";
                setV4BPosition(ARM_RIGHT_POS);
                if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    resetV4B();
                }
                break;
            case MEGA_REST:
                armDisplayText = "Mega Rest";
                setV4BPosition(MEGA_REST_POS);
                if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) {
                    armState = ArmState.REST;
                }
        }

        // The Specimen Arm should interact closely with the Virtual Four Bar.
        switch (specimenState) {
            case INTAKE:
                if (specimenTimer.seconds() > 0.2) {
                    specimenWrist.setPosition(0.5 + SPECIMEN_WRIST_INTAKE_ANGLE);
                } else {
                    specimenWrist.setPosition(0.5 + SPECIMEN_WRIST_TRANSITION_ANGLE);
                }
                specimenBar.setPosition(0.5 + SPECIMEN_BAR_INTAKE_ANGLE);
                break;
            case OUTTAKE:
                specimenWrist.setPosition(0.5 + SPECIMEN_WRIST_OUTTAKE_ANGLE);
                specimenBar.setPosition(0.5 + SPECIMEN_BAR_OUTTAKE_ANGLE);
                break;
        }

        switch (wristState) {
            /*
            Going down...
                Up          |
                Neutral <-  |
                Low      |  |
                Down ----^  v
             Wrist timer controlled by setWristState. For delays when needed.
             */
            case NEUTRAL:
                wristDisplayText = "Neutral";
                if (wristTimer.seconds() > 0.5)
                    intakeWrist.setPosition(WRIST_PICKUP);
                if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    wristState = WristState.LOW;
                }
                break;
            case UP:
                wristDisplayText = "Up";
                if (wristTimer.seconds() > 0.5)
                    intakeWrist.setPosition(WRIST_UP);
                if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    wristState = WristState.NEUTRAL;
                }
                break;
            case LOW:
                wristDisplayText = "Low";
                if (wristTimer.seconds() > 0.5)
                    intakeWrist.setPosition(WRIST_PICKUP_LOW);
                if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    wristState = WristState.DOWN;
                }
                break;
            case DOWN:
                wristDisplayText = "Down";
                if (wristTimer.seconds() > 0.5)
                    intakeWrist.setPosition(WRIST_DOWN);
                if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    wristState = WristState.NEUTRAL;
                }
                break;
            case SCORE:
                wristDisplayText = "Scoring";
                if (wristTimer.seconds() > 0.5)
                    intakeWrist.setPosition(WRIST_SCORE);
                break;
        }
    }

    public void runIntake(Gamepad gamepad) {
        switch (intakeState) {
            case IDLE:
                intakeDisplayText = "IDLE";
                setIntakePowers(0);

                if (gamepad.left_trigger > 0) {
                    intaked = false;
                    intakeState = IntakeState.OUT;
                } else if (gamepad.right_trigger > 0) {
                    if (wristState == WristState.DOWN || wristState == WristState.LOW) {
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
                    intaked = true;
                    if (wristState == WristState.DOWN || wristState == WristState.LOW)
                        slideState = SlideState.REST;
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
        }
    }

    // Additional Hanging Code
    public void runHang(GamepadEx gamepad) {
        switch (hangState) {
            case REST:
                // Arm subsystem normal functionality, linear actuator down.
                hangDisplayText = "REST";

                if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
                    hangState = HangState.READY;
                    intakeState = IntakeState.HANG; // Prevent any outside control from intake loop
                    stallTimer.reset(); // Prevent stalling when moving slides to REST
                }
                break;
            case READY:
                hangDisplayText = "X to cancel";
//                slideState = SlideState.INTAKE; would break this due to encoder constant resetting
                wristState = WristState.UP;

                // DPAD Down to Ascend Level 2
                if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
                    slideState = SlideState.HANG; // Prevent any outside control from slide loop
                    slidePower = 1; // More power to hang.
//                    targetSlidePosition = HANG_POSITION_SLIDES; Disabled for meet 2
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

                /* Disabled for Meet 2
                // Manual adjust worm gear
                if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    wormMotor.setPower(0.8);
                } else if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    wormMotor.setPower(-0.8);
                } else {
                    wormMotor.setPower(0);
                }
                 */

                /* Disabled for meet 2
                // DPAD Up to start Ascend Level 3
                if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    targetSlidePosition = HANG_POSITION_SLIDES - 1500;
                    hangMotor.setTargetPosition(HANG_LINEAR_REST);
                    hangState = HangState.LEVEL3;
                } else
                 */
                if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                    hangState = HangState.READY;
                    slidePower = 0.5;
                }
                break;
            case LEVEL3:
                hangDisplayText = "Ascended!!!";

                // Move the slides up or down
                if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    targetSlidePosition -= (MANUAL_INCREMENT+20);
                } else if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    targetSlidePosition += (MANUAL_INCREMENT+20);
                }
        }
    }

    public void runSubsystem(GamepadEx gamepadEx, Gamepad gamepad) {
        runSlides(gamepadEx);
        runArm(gamepadEx);
        runIntake(gamepad);
        runHang(gamepadEx);
    }
}
