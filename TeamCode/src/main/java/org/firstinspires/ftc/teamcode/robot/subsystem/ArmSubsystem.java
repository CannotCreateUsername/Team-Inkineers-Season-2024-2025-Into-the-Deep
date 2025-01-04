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

/** @noinspection FieldCanBeLocal */
@Config
public abstract class ArmSubsystem {

    // Standard Rotations for Servos...
    private final double MAX_AXON_ROTATION = 180.98;
    private final double MAX_GOBILDA_ROTATION = 300;

    public enum SlideState {
        REST,
        OUTTAKE,
        HANG
    }

    public enum ArmState {
        REST,
        MEGA_REST,
        LEFT,
        RIGHT,
        INTAKE,
        HANG
    }

    public enum SpecimenState {
        INTAKE,
        OUTTAKE, // The state right before scoring
        AUTO,
        HANG
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
        DROPOFF,
        SCORE,
        LOW
    }

    public enum HangState {
        REST,
        READY,
        LEVEL2,
        LEVEL3
    }

    public enum AreaState {
        CLOSE,
        FAR
    }

    SlideState slideState;
    ArmState armState;
    SpecimenState specimenState;
    IntakeState intakeState;
    WristState wristState;
    HangState hangState;
    AreaState areaState;

    public int targetSlidePosition;

    // Constants, needs to be adjusted based on testing
    // Linear Slides
    final int REST_POSITION_SLIDES = 0;
    public static int OUTTAKE_POSITION_SLIDES = 2500;
    public static int HANG_POSITION_SLIDES = 2500;
    final int MAX_EXTEND_POSITION = 3000;
    final int MANUAL_INCREMENT = 40;
    public static double DEFAULT_SLIDE_POWER = 1;

    private final double MAX_INTAKE_WRIST_ROTATION = 236.0; // The new neutral. 12/7/24

    final double WRIST_NEUTRAL = 0.5;
    final double WRIST_UP = WRIST_NEUTRAL + 120.0/MAX_INTAKE_WRIST_ROTATION;
    final double WRIST_DOWN = WRIST_NEUTRAL - 120.0/MAX_INTAKE_WRIST_ROTATION;
    final double WRIST_SCORE = WRIST_NEUTRAL - 25.0/MAX_INTAKE_WRIST_ROTATION;
    final double WRIST_DROPOFF = WRIST_NEUTRAL - 40.0/MAX_INTAKE_WRIST_ROTATION;
    final double WRIST_PICKUP = WRIST_NEUTRAL - 20.0/MAX_INTAKE_WRIST_ROTATION;
    final double WRIST_PICKUP_LOW = WRIST_NEUTRAL - 70.0/MAX_INTAKE_WRIST_ROTATION;

    // Coaxial V4B positions
    // Lower servos. Axon, standard rotation of 180.98 degrees.
    private final double MAX_LOWER_BAR_ROTATION = 200.5; // 144/255
    final double V4B_LOWER_CENTER = 0.5;
    final double V4B_LOWER_REST = V4B_LOWER_CENTER + 80.0/MAX_LOWER_BAR_ROTATION;
    final double V4B_LOWER_LEFT = V4B_LOWER_CENTER - 90.0/MAX_LOWER_BAR_ROTATION;
    final double V4B_LOWER_RIGHT = V4B_LOWER_CENTER + 90.0/MAX_LOWER_BAR_ROTATION;
    final double V4B_LOWER_INITIAL = V4B_LOWER_CENTER + 70.0/MAX_LOWER_BAR_ROTATION;

    // Upper servo. Axon, standard rotation of 180.98 degrees.
    private final double MAX_UPPER_BAR_ROTATION = 236.7; // 170/255
    final double V4B_UPPER_CENTER = 0.5;
    final double V4B_UPPER_LEFT = V4B_UPPER_CENTER - 100.0/MAX_UPPER_BAR_ROTATION;
    final double V4B_UPPER_REST = V4B_UPPER_CENTER - 80.0/MAX_UPPER_BAR_ROTATION;
//    final double V4B_UPPER_TRANSITION = V4B_UPPER_LEFT - 20.0/MAX_UPPER_BAR_ROTATION;
    final double V4B_UPPER_RIGHT = V4B_UPPER_CENTER + 100.0/MAX_UPPER_BAR_ROTATION;

    // 0 is lower servo position. 1 is upper servo position.
    final double[] ARM_LEFT_POS = {V4B_LOWER_CENTER, V4B_UPPER_LEFT, WRIST_NEUTRAL};
    final double[] ARM_RIGHT_POS = {V4B_LOWER_CENTER, V4B_UPPER_RIGHT, WRIST_NEUTRAL};
    final double[] ARM_REST_POS = {V4B_LOWER_REST, V4B_UPPER_LEFT, WRIST_NEUTRAL};
    final double[] ARM_INTAKE_POS = {V4B_LOWER_CENTER, V4B_UPPER_CENTER, WRIST_DOWN};
    final double[] MEGA_REST_POS = {V4B_LOWER_CENTER, V4B_UPPER_CENTER, WRIST_NEUTRAL};

    // Specimen Actuator Positions.
    private final double MAX_SPECIMEN_BAR_ROTATION = 355;
    private final double MAX_SPECIMEN_WRIST_ROTATION = MAX_GOBILDA_ROTATION;

    final double SPECIMEN_BAR_NEUTRAL = 0.5;
    final double SPECIMEN_BAR_INITIAL_ANGLE = SPECIMEN_BAR_NEUTRAL +(70.0+90.0)/MAX_SPECIMEN_BAR_ROTATION;
    final double SPECIMEN_BAR_INTAKE_ANGLE = SPECIMEN_BAR_NEUTRAL +(70.0+90.0)/MAX_SPECIMEN_BAR_ROTATION;
    final double SPECIMEN_BAR_OUTTAKE_ANGLE = SPECIMEN_BAR_NEUTRAL -45.0/MAX_SPECIMEN_BAR_ROTATION;
    final double SPECIMEN_BAR_STRAIGHT_ANGLE = SPECIMEN_BAR_NEUTRAL -90.0/MAX_SPECIMEN_BAR_ROTATION;

    final double SPECIMEN_WRIST_NEUTRAL = 0.5;
    final double SPECIMEN_WRIST_INITIAL_ANGLE = SPECIMEN_WRIST_NEUTRAL +(90.0+30.0)/MAX_SPECIMEN_WRIST_ROTATION;
    final double SPECIMEN_WRIST_INTAKE_ANGLE = SPECIMEN_WRIST_NEUTRAL +60.0/MAX_SPECIMEN_WRIST_ROTATION;
//    final double SPECIMEN_WRIST_OUTTAKE_ANGLE = SPECIMEN_WRIST_NEUTRAL +70.0/MAX_SPECIMEN_WRIST_ROTATION;
    final double SPECIMEN_WRIST_TRANSITION_OFF = SPECIMEN_WRIST_NEUTRAL -30.0/MAX_SPECIMEN_WRIST_ROTATION;
    final double SPECIMEN_WRIST_TRANSITION_ON = SPECIMEN_WRIST_NEUTRAL +60.0/MAX_SPECIMEN_WRIST_ROTATION;

    // Worm Gear
    // Manually Controlled
    // final int HANG_WORM_READY = 500;

    // Declare actuator variables
    public List<DcMotorEx> slideMotors; // Initialize as list to support potential multiple motors
    public List<CRServo> intakeServos;
    public List<Servo> lowerBar;
    public Servo upperBar;
    public Servo intakeWrist;
    public Servo specimenBar;
    public Servo specimenWrist;

    public DcMotor wormMotor;

    // Sensors
    public ColorSensor racist;
    public TouchSensor slideSwitch;

    boolean redSide = false;
    // Overarching Initialization Method
    public void init(HardwareMap hardwareMap, boolean isRedAlliance, boolean manualTesting) {
        initSlideSystem(hardwareMap);

        // Activate all other subsystems if not manual testing
        if (!manualTesting) {
            initIntake(hardwareMap);
            initV4B(hardwareMap);
            initSpecimen(hardwareMap);
            initSensors(hardwareMap, isRedAlliance);

            // Initialize Positions; Start at REST
            slideState = SlideState.REST;
            armState = ArmState.REST;
            specimenState = SpecimenState.OUTTAKE;
            intakeState = IntakeState.IDLE;
            wristState = WristState.UP;
            hangState = HangState.REST;
            areaState = AreaState.CLOSE;

            targetSlidePosition = REST_POSITION_SLIDES;
            intakeWrist.setPosition(WRIST_UP);
            setV4BPosition(V4B_LOWER_INITIAL, V4B_UPPER_REST);
            specimenBar.setPosition(SPECIMEN_BAR_INITIAL_ANGLE);
            specimenWrist.setPosition(SPECIMEN_WRIST_INITIAL_ANGLE);
        }
    }
    public void init(HardwareMap hardwareMap, boolean isRedAlliance) {
        init(hardwareMap, isRedAlliance, false);
    }
    // For manual adjustment in a separate OpMode
    public void initManualTesting(HardwareMap hardwareMap) {
        init(hardwareMap, false, true);
        initIntake(hardwareMap);
        initV4B(hardwareMap);
        intakeWrist.setPosition(WRIST_UP);
        setV4BPosition(V4B_LOWER_INITIAL, V4B_UPPER_REST);
    }

    // Initializing Individually

    public void initSlideSystem(HardwareMap hardwareMap) {
        slideMotors = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, "left_slide"),
                hardwareMap.get(DcMotorEx.class, "right_slide")
        );
        // Set behavior for each motor
        for (DcMotorEx m : slideMotors) {
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        // Reverse a slide motor. Depends on orientation.
        slideMotors.get(1).setDirection(DcMotorSimple.Direction.REVERSE);

        // Worm Gear
        wormMotor = hardwareMap.get(DcMotor.class, "worm_motor");
        // Worm Motor Behavior
        wormMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wormMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initIntake(HardwareMap hardwareMap) {
        intakeServos = Arrays.asList(
                hardwareMap.get(CRServo.class, "intake_left"),
                hardwareMap.get(CRServo.class, "intake_right")
        );
        // Reverse the right intake servo
        intakeServos.get(1).setDirection(DcMotorSimple.Direction.REVERSE);

        intakeWrist = hardwareMap.get(Servo.class, "intake_wrist");
    }

    public void initV4B(HardwareMap hardwareMap) {
        lowerBar = Arrays.asList(
                hardwareMap.get(Servo.class, "lower_bar")
        );
        upperBar = hardwareMap.get(Servo.class, "upper_bar");
        // Reverse V4B servos, upper bar
        for (Servo s : lowerBar) {
            s.setDirection(Servo.Direction.FORWARD);
        }
        upperBar.setDirection(Servo.Direction.REVERSE);
    }

    public void initSpecimen(HardwareMap hardwareMap) {
        specimenBar = hardwareMap.get(Servo.class, "specimen_bar");
        specimenWrist = hardwareMap.get(Servo.class, "specimen_wrist");
        specimenWrist.setDirection(Servo.Direction.REVERSE);
    }

    public void initSensors(HardwareMap hardwareMap, boolean isRedAlliance) {
        racist = hardwareMap.get(ColorSensor.class, "racist");
        redSide = isRedAlliance;
        slideSwitch = hardwareMap.get(TouchSensor.class, "slide_limit");
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
    // Method to set power to slide motors manually
    public void setSlidePowers(double power) {
        for (DcMotorEx m : slideMotors) {
            m.setPower(power);
        }
    }
    // Method to reset slide encoders
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
    public void setArmState(ArmState state) {
        if (state == ArmState.REST) {
            armTimer.reset();
        }
        armState = state;
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

    public ElapsedTime specimenTimer = new ElapsedTime();
    public void setSpecimenState(SpecimenState state) {
        if (specimenState == SpecimenState.OUTTAKE && state != SpecimenState.OUTTAKE) {
            specimenTimer.reset(); // Allow to go to transition state to score
        }
        specimenState = state;
    }

    // RGB for yellow is (255, 255, 0)
    public String slideDisplayText = "WEEWOOWEEWOOWEEWOOWEEWOOWEEEEEEEEEEEEEEEEEEEEEEE";
    public String armDisplayText = "HEEEEEEEEEEEEEEEEEEEEYAWxd";
    public String intakeDisplayText = "NOMMMMMMMMMMMMMMMMM";
    public String wristDisplayText = "YEEEEEEEEEEEEEEET";
    public String hangDisplayText = "RRRRRRRRRRRRRRRRe";
}
