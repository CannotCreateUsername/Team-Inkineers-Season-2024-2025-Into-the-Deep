package org.firstinspires.ftc.teamcode.robot.subsystem;

import static org.firstinspires.ftc.teamcode.robot.constants.PIDConstants.kPslides;
import static org.firstinspires.ftc.teamcode.robot.constants.PIDConstants.threshold_slides;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
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

    private enum IntakeState {
        IDLE,
        IN,
        OUT
    }

    private enum WristState {
        NEUTRAL,
        UP,
        DOWN
    }

    private enum HangState {
        REST,
        READY,
        HANGING
    }

    ArmState armState;
    IntakeState intakeState;
    WristState wristState;
    HangState hangState;

    public int targetSlidePosition;

    // Needs to be adjusted based on testing
    // Linear Slides
    private final int REST_POSITION_SLIDES = 0;
    private final int INTAKE_POSITION_SLIDES = 1000;
    private final int OUTTAKE_POSITION_SLIDES = 2000;
    private final int MAX_EXTEND_POSITION = 3500;
    private final int MANUAL_INCREMENT = 20;

    // Declare actuator variables
    private final List<DcMotorEx> slideMotors; // Initialize as list to support potential multiple motors

    private final Servo wrist;
    private final CRServo intake;
    private final CRServo intake2;

    private final DcMotorEx hangMotor;
    private final Servo hangServo;

    // Constructor :]
    public ArmSubsystem1(HardwareMap hardwareMap) {
        // Map the actuators
        slideMotors = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, "slide1"),
                hardwareMap.get(DcMotorEx.class, "slide2")
        );
        intake = hardwareMap.get(CRServo.class, "intake");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
        wrist = hardwareMap.get(Servo.class, "wrist");
        hangMotor = hardwareMap.get(DcMotorEx.class, "hang_motor");
        hangServo = hardwareMap.get(Servo.class, "hang_servo");

        // Set Motor Modes & Directions
        for (DcMotorEx m : slideMotors) {
            m.setDirection(DcMotorSimple.Direction.REVERSE);
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

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

    // Method to simplify getting the current linear slides position
    public int getSlidesPosition() {
        return slideMotors.get(0).getCurrentPosition();
    }

    public void runArm(GamepadEx gamepad) {
        // Arm control logic
        switch (armState) {
            case REST:
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
                }
                break;
            case OUTTAKE:
                slideDisplayText = "OUTTAKE";
                targetSlidePosition = OUTTAKE_POSITION_SLIDES;
                if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    targetSlidePosition -= MANUAL_INCREMENT;
                } else if (gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
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
        runSlideMotors(0.5);
    }

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
                if (gamepad.right_trigger > 0) {
                    intakeState = IntakeState.IN;
                    if (armState == ArmState.REST) {
                        wristState = WristState.DOWN;
                    }
                }
                if (gamepad.left_trigger > 0) {
                    intakeState = IntakeState.OUT;
                    wristState = WristState.NEUTRAL;
                }
                break;
            case IN:
                intake.setPower(1);
                intake2.setPower(1);
                intakeDisplayText = "IN";

                if (gamepad.right_trigger == 0) {
                    intakeState = IntakeState.IDLE;
                }
                break;
            case OUT:
                intake.setPower(-1);
                intake2.setPower(-1);
                intakeDisplayText = "OuT";

                if (gamepad.left_trigger == 0) {
                    intakeState = IntakeState.IDLE;
                }
                break;
        }
        runWrist();
    }

    // Max Rotation for 2000-0025-0002 Torque Servo: 300 degrees
    // 90 degrees is position +- 90/300
    private final double WRIST_NEUTRAL = 0.5;
    private final double WRIST_UP = 0.5+90.0/300;
    private final double WRIST_DOWN = 0.5-90.0/300;

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

    private final int HANG_UP = 3000;
    private final int HANG_DOWN = 1000;
    private final int HANG_REST = 0;

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

    public String slideDisplayText = "WEEWOOWEEWOOWEEWOOWEEWOOWEEEEEEEEEEEEEEEEEEEEEEE";
    public String intakeDisplayText = "NOMMMMMMMMMMMMMMMMM";
    public String wristDisplayText = "YEEEEEEEEEEEEEEET";

    // Autonomous Actions for RoadRunner

    ElapsedTime autoTimer = new ElapsedTime();
    public Action spinIntake() {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    autoTimer.reset();
                    set = true;
                }
                intake.setPower(1);
                return autoTimer.seconds() < 2;
            }
        };
    }
}
