package org.firstinspires.ftc.teamcode.robot.subsystem;

import static org.firstinspires.ftc.teamcode.robot.constants.PIDConstants.kDslides;
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
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

/** @noinspection FieldCanBeLocal*/
@Config
public abstract class ArmSubsystem {
    public enum SlideState {
        REST,
        INTAKE,
        OUTTAKE,
        HANG
    }

    public enum ArmState {
        REST,
        LEFT_FAR,
        RIGHT_FAR,
        MEGA_REST
    }

    public enum AreaState {
        CLOSE,
        FAR
    }

    public enum IntakeState {
        IDLE,
        IN,
        OUT,
        HANG
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
        LEVEL2,
        LEVEL3
    }

    SlideState slideState;
    ArmState armState;
    AreaState areaState;
    IntakeState intakeState;
    WristState wristState;
    HangState hangState;

    public int targetSlidePosition;

    // Constants, needs to be adjusted based on testing
    // Linear Slides
    final int REST_POSITION_SLIDES = 0;
    public static int INTAKE_POSITION_SLIDES = 450;
    public static int OUTTAKE_POSITION_SLIDES = 2460;
    public static int HANG_POSITION_SLIDES = 2500;
    final int MAX_EXTEND_POSITION = 3000;
    final int MANUAL_INCREMENT = 40;
    public static double DEFAULT_SLIDE_POWER = 1;

    // Default Rotation for Axon MAX+ Servo: 180 degrees
    // 90 degrees is position +- 90/180.9
    final double WRIST_NEUTRAL = 0.5;
    final double WRIST_UP = WRIST_NEUTRAL + 120.0/236.0;
    final double WRIST_DOWN = WRIST_NEUTRAL - 120.0/236.0;
    final double WRIST_SCORE = WRIST_NEUTRAL - 25.0/236.0;
    final double WRIST_PICKUP = WRIST_NEUTRAL - 20.0/236.0; // The new neutral. 12/7/24

    // Coaxial V4B positions
    // Lower servos. Axon, standard rotation of 180.98 degrees.
    final double V4B_LOWER_CENTER = 0.5;
    final double V4B_LOWER_LEFT = V4B_LOWER_CENTER - 90.0/180;
    final double V4B_LOWER_RIGHT = V4B_LOWER_CENTER + 90.0/180;

    // Upper servo. Axon, max rotation of 180.9 degrees.
    final double V4B_UPPER_CENTER = 0.5;
    final double V4B_UPPER_LEFT = V4B_UPPER_CENTER - 105.0/236.0;
    final double V4B_UPPER_REST = V4B_UPPER_CENTER - 64.0/236.0;
    final double V4B_UPPER_TRANSITION = V4B_UPPER_LEFT - 20.0/236.0;
    final double V4B_UPPER_RIGHT = V4B_UPPER_CENTER + 90.0/236.0;

    // 0 is lower servo position. 1 is upper servo position.
    final double[] ARM_LEFT_POS = {V4B_LOWER_LEFT, V4B_UPPER_LEFT, WRIST_DOWN};
    final double[] ARM_REST_POS = {V4B_LOWER_RIGHT, V4B_UPPER_REST, WRIST_UP};
    final double[] ARM_RIGHT_POS = {V4B_LOWER_RIGHT, V4B_UPPER_RIGHT, WRIST_DOWN};
    final double[] MEGA_REST_POS = {V4B_LOWER_CENTER, V4B_UPPER_CENTER, WRIST_NEUTRAL};


    // Linear Actuator
    final int HANG_LINEAR_REST = 0;
    final int HANG_LINEAR_UP = 2900;
    final int HANG_LINEAR_DOWN = 1100;
    // Worm Gear
    // Manually Controlled
    // final int HANG_WORM_READY = 500;

    // Declare actuator variables
    public List<DcMotorEx> slideMotors; // Initialize as list to support potential multiple motors
    public List<CRServo> intakeServos;
    public List<Servo> lowerBar;
    public Servo upperBar;
    public Servo wrist;

    public DcMotor hangMotor;
    public DcMotor wormMotor;
    public Servo latchServo;


    // Sensors
    public ColorSensor racist;
    public TouchSensor slideSwitch;

    boolean redSide = false;
    /** @noinspection ArraysAsListWithZeroOrOneArgument*/
    public void init(HardwareMap hardwareMap, boolean isRedAlliance) {
        // For color sensor
        redSide = isRedAlliance;
        // Map the actuators
        slideMotors = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, "left_slide"),
                hardwareMap.get(DcMotorEx.class, "right_slide")
        );
        intakeServos = Arrays.asList(
                hardwareMap.get(CRServo.class, "intake")
        );
        lowerBar = Arrays.asList(
                hardwareMap.get(Servo.class, "lower_bar")
        );
        upperBar = hardwareMap.get(Servo.class, "upper_bar");
        wrist = hardwareMap.get(Servo.class, "wrist");

        hangMotor = hardwareMap.get(DcMotor.class, "hang_motor");
        wormMotor = hardwareMap.get(DcMotor.class, "worm_motor");
        latchServo = hardwareMap.get(Servo.class, "latch");

        // Map sensors
        racist = hardwareMap.get(ColorSensor.class, "racist");
        slideSwitch = hardwareMap.get(TouchSensor.class, "slide_limit");

        // Set Motor Modes & Directions
        // Reverse slide motors. Depends on orientation of Bevel Gear
        for (DcMotorEx m : slideMotors) {
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Reverse right intake servo.
        // intakeServos.get(0).setDirection(DcMotorSimple.Direction.REVERSE);
        // Reverse wrist servo
        wrist.setDirection(Servo.Direction.REVERSE);
        // Reverse V4B servos, upper bar.
        upperBar.setDirection(Servo.Direction.REVERSE);

        // Hanging Stuff
        hangMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wormMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wormMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize Positions; Start at REST
        slideState = SlideState.REST;
        armState = ArmState.REST;
        areaState = AreaState.CLOSE;
        intakeState = IntakeState.IDLE;
        wristState = WristState.UP;
        hangState = HangState.REST;

        targetSlidePosition = INTAKE_POSITION_SLIDES;
        wrist.setPosition(WRIST_UP);
        setV4BPosition(V4B_LOWER_RIGHT, V4B_UPPER_REST);
    }

    // For manual adjustment in a separate OpMode
    public void initManualTesting(HardwareMap hardwareMap) {
        slideMotors = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, "left_slide"),
                hardwareMap.get(DcMotorEx.class, "right_slide")
        );
        hangMotor = hardwareMap.get(DcMotor.class, "hang_motor");
        wormMotor = hardwareMap.get(DcMotor.class, "worm_motor");

        // Set Motor Modes & Directions
        for (DcMotorEx m : slideMotors) {
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Hanging Stuff
        hangMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wormMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wormMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
                m.setPower((slideError * kPslides + kDslides)*power);
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


    // Method to move the arm.
    /* Order:
     * 0: Lower Bar
     * 1: Upper Bar
     * 2: Wrist
     */
    public void setV4BPosition(double[] position) {
        if (position.length > 0) {
            for (Servo s : lowerBar) {
                s.setPosition(position[0]);
            }
        }
        if (position.length > 1) {
            upperBar.setPosition(position[1]);
        }
    }

    ElapsedTime armTimer = new ElapsedTime();
    public void resetV4B() {
        armTimer.reset();
        wristState = WristState.UP;
        if (armState == ArmState.LEFT_FAR) {
            setV4BPosition(V4B_LOWER_RIGHT, V4B_UPPER_TRANSITION);
        }
        armState = ArmState.REST;
    }
    // Used in manual TeleOp testing
    public void setV4BPosition(double lowerPos, double upperPos) {
        for (Servo s : lowerBar) {
            s.setPosition(lowerPos);
        }
        upperBar.setPosition(upperPos);
    }


    public ElapsedTime wristTimer = new ElapsedTime();
    public void setWristState(WristState wState, boolean delayed) {
        if (delayed) {
            wristTimer.reset();
        }
        wristState = wState;
    }

    public boolean getInvalidColor() {
        // Yellow samples are more red, make sure difference is great to pick up yellow
        if (redSide) {
            return racist.blue() > 200 && racist.blue() > 3.5*racist.red();
        } else {
            return racist.red() > 200 && racist.red() > 3.5*racist.blue();
        }
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
    public String armDisplayText = "HEEEEEEEEEEEEEEEEEEEEYAWxd";
    public String intakeDisplayText = "NOMMMMMMMMMMMMMMMMM";
    public String wristDisplayText = "YEEEEEEEEEEEEEEET";
    public String hangDisplayText = "RRRRRRRRRRRRRRRRe";
}
