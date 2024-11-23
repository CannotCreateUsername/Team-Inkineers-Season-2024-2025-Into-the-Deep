package org.firstinspires.ftc.teamcode.robot.subsystem;

import static org.firstinspires.ftc.teamcode.robot.constants.PIDConstants.kPslides;
import static org.firstinspires.ftc.teamcode.robot.constants.PIDConstants.threshold_slides;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

/** @noinspection FieldCanBeLocal*/
@Config
public abstract class ArmSubsystem {
    public enum ArmState {
        REST,
        INTAKE,
        OUTTAKE,
    }

    public enum IntakeState {
        IDLE,
        IN,
        OUT
    }

    public enum WristState {
        NEUTRAL,
        UP,
        DOWN,
        SCORE
    }

    public enum HangState {
        REST,
        READY,
        HANGING
    }

    ArmState armState;
    IntakeState intakeState;
    WristState wristState;
    HangState hangState;

    public int targetSlidePosition;

    // Constants, needs to be adjusted based on testing
    // Linear Slides
    final int REST_POSITION_SLIDES = 0;
    final int INTAKE_POSITION_SLIDES = 270;
    final int OUTTAKE_POSITION_SLIDES = 2460;
    final int MAX_EXTEND_POSITION = 3000;
    final int MANUAL_INCREMENT = 40;

    // Default Rotation for Axon MAX+ Servo: 180 degrees
    // 90 degrees is position +- 90/180.9
    final double WRIST_NEUTRAL = 0.5;
    final double WRIST_UP = WRIST_NEUTRAL+90.0/180.9;
    final double WRIST_DOWN = WRIST_NEUTRAL-72.0/180.9;
    final double WRIST_SCORE = WRIST_NEUTRAL-25.0/180.9;

    // Coaxial V4B positions
    // TODO: find servo rotation range for upper and lower V4B actuators
    final double V4B_CENTER_A = 0.5,                    V4B_CENTER_B = 0.5;
    final double V4B_LEFT_A = V4B_CENTER_A-90.0/180.9,  V4B_LEFT_B = V4B_CENTER_B-90.0/180.9;
    final double V4B_RIGHT_A = V4B_CENTER_A+90.0/180.9, V4B_RIGHT_B = V4B_CENTER_B+90.0/180.9;;

    // Linear Actuator
    final int HANG_2_UP = 3050;
    final int HANG_2_DOWN = 1100;
    final int HANG_2_REST = 0;
    // Worm Gear
    // TODO: fine tune worm gear positions
    final int HANG_3_REST = 0;
    final int HANG_3_UP = 1000;

    // Declare actuator variables
    public List<DcMotorEx> slideMotors; // Initialize as list to support potential multiple motors
    public List<CRServo> intakeServos;
    public List<Servo> lowerBar;
    public Servo upperBar;
    public Servo wrist;

    public DcMotor hangMotor;
    public DcMotor wormMotor;
    public Servo hangServo;

    public ColorSensor racist;

    boolean redSide = false;
    /** @noinspection ArraysAsListWithZeroOrOneArgument*/
    public void init(HardwareMap hardwareMap, boolean isRedAlliance) {
        // For color sensor
        redSide = isRedAlliance;
        // Map the actuators
        slideMotors = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, "belt_slide"),
                hardwareMap.get(DcMotorEx.class, "non_slide")
        );
        intakeServos = Arrays.asList(
                hardwareMap.get(CRServo.class, "right"),
                hardwareMap.get(CRServo.class, "left")
        );
        lowerBar = Arrays.asList(
                hardwareMap.get(Servo.class, "lower_bar")
        );
        upperBar = hardwareMap.get(Servo.class, "upper_bar");
        wrist = hardwareMap.get(Servo.class, "wrist");

        hangMotor = hardwareMap.get(DcMotor.class, "hang_motor");
        wormMotor = hardwareMap.get(DcMotor.class, "worm_motor");
        hangServo = hardwareMap.get(Servo.class, "hang_servo");

        // Map sensors
        racist = hardwareMap.get(ColorSensor.class, "racist");

        // Set Motor Modes & Directions
        // Reverse slide motors. Depends on orientation of Bevel Gear
        for (DcMotorEx m : slideMotors) {
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            m.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Reverse right intake servo
        intakeServos.get(0).setDirection(DcMotorSimple.Direction.REVERSE);
        // Reverse wrist servo
        wrist.setDirection(Servo.Direction.REVERSE);
        // TODO: Reverse V4B servos if needed. Bottom and top.

        // Hanging Stuff
        hangMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wormMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize Positions; Start at REST
        armState = ArmState.REST;
        intakeState = IntakeState.IDLE;
        wristState = WristState.NEUTRAL;
        hangState = HangState.REST;

        targetSlidePosition = INTAKE_POSITION_SLIDES;
        wrist.setPosition(WRIST_UP);
    }

    // For manual adjustment in a separate OpMode
    public void initManualTesting(HardwareMap hardwareMap) {
        slideMotors = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, "belt_slide"),
                hardwareMap.get(DcMotorEx.class, "non_slide")
        );
        hangMotor = hardwareMap.get(DcMotor.class, "hang_motor");
        wormMotor = hardwareMap.get(DcMotor.class, "worm_motor");

        // Set Motor Modes & Directions
        // TODO: Reverse slide motor encoders. Depends on orientation of Bevel Gear
        for (DcMotorEx m : slideMotors) {
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            m.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Hanging Stuff
        hangMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wormMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Method to run intake servos
    public void setIntakePowers(double power) {
        for (CRServo s : intakeServos) {
            s.setPower(power);
        }
    }

    // Method to run slide motors to position
    double slideError;
    public void runSlideMotorsPID(double power) {
        slideError = targetSlidePosition - getSlidesPosition();
        for (DcMotorEx m : slideMotors) {
            if (Math.abs(slideError) > threshold_slides) {
                m.setPower((slideError * kPslides)*power);
            } else {
                m.setPower(0.0);
            }
        }
    }

    public void resetSlideEncoders() {
        for (DcMotorEx m : slideMotors) {
            m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // Method to simplify getting the current linear slides position
    public int getSlidesPosition() {
        return slideMotors.get(0).getCurrentPosition();
    }

    public boolean getInvalidColor() {
        // Yellow samples are more red, make sure difference is great to pick up yellow
        if (redSide) {
            return racist.blue() > 200 && racist.blue() > 3.5*racist.red();
        } else {
            return racist.red() > 200 && racist.red() > 3.5*racist.blue();
        }
    }

    public boolean intakedSpecial() {
        return (racist.red() > 200) || (racist.blue() > 200);
    }

    boolean eject = false;
    ElapsedTime ejectTimer = new ElapsedTime();
    public void eject() {
        eject = true;
        ejectTimer.reset();
        intakeState = IntakeState.OUT;
        wristState = WristState.NEUTRAL;
    }

    public String slideDisplayText = "WEEWOOWEEWOOWEEWOOWEEWOOWEEEEEEEEEEEEEEEEEEEEEEE";
    public String intakeDisplayText = "NOMMMMMMMMMMMMMMMMM";
    public String wristDisplayText = "YEEEEEEEEEEEEEEET";
    public String hangDisplayText = "RRRRRRRRRRRRRRRRe";
}
