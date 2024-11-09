package org.firstinspires.ftc.teamcode.robot.subsystem;

import static org.firstinspires.ftc.teamcode.robot.constants.PIDConstants.kPslides;
import static org.firstinspires.ftc.teamcode.robot.constants.PIDConstants.threshold_slides;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    ArmState armState;
    IntakeState intakeState;
    WristState wristState;

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

        hangMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize Positions; Start at REST
        armState = ArmState.REST;
        intakeState = IntakeState.IDLE;
        wristState = WristState.UP;

        targetSlidePosition = REST_POSITION_SLIDES;
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
                slideDisplayText = "Retracted";
                targetSlidePosition = REST_POSITION_SLIDES;
                if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    armState = ArmState.INTAKE;
                }
                break;
            case INTAKE:
                slideDisplayText = "Intake";
                targetSlidePosition = INTAKE_POSITION_SLIDES;
                if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    armState = ArmState.OUTTAKE;
                }
                break;
            case OUTTAKE:
                slideDisplayText = "Outtake";
                targetSlidePosition = OUTTAKE_POSITION_SLIDES;
                if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    targetSlidePosition -= MANUAL_INCREMENT;
                }
                if (gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
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

    public void runIntake(GamepadEx gamepad) {
        TriggerReader rtReader = new TriggerReader(gamepad, GamepadKeys.Trigger.RIGHT_TRIGGER);
        TriggerReader ltReader = new TriggerReader(gamepad, GamepadKeys.Trigger.LEFT_TRIGGER);
        switch (intakeState) {
            case IDLE:
                // Default Wrist States
                if (armState == ArmState.REST) {
                    wristState = WristState.UP;
                } else {
                    wristState = WristState.NEUTRAL;
                }
                if (rtReader.isDown() && armState == ArmState.REST) {
                    intakeState = IntakeState.IN;
                    wristState = WristState.DOWN;
                } else if (rtReader.isDown()) {
                    intakeState = IntakeState.IN;
                }
                if (ltReader.isDown()) {
                    intakeState = IntakeState.OUT;
                    wristState = WristState.NEUTRAL;
                }
                break;
            case IN:
                intake.setPower(1);
                if (rtReader.wasJustReleased()) {
                    intakeState = IntakeState.IDLE;
                }
                break;
            case OUT:
                intake.setPower(-1);
                if (ltReader.wasJustReleased()) {
                    intakeState = IntakeState.IDLE;
                }
                break;
        }
        runWrist();
    }

    private void runWrist() {
        switch (wristState) {
            case NEUTRAL:
                wrist.setPosition(0.5);
                break;
            case UP:
                wrist.setPosition(1);
                break;
            case DOWN:
                wrist.setPosition(0);
                break;
        }
    }

    public String slideDisplayText = "WEEWOOWEEWOOWEEWOOWEEWOOWEEEEEEEEEEEEEEEEEEEEEEE";


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
