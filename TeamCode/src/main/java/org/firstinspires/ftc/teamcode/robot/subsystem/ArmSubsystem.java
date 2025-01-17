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
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

/** @noinspection FieldCanBeLocal */
@Config
public abstract class ArmSubsystem {

    // Standard Rotations for Servos...
    // private final double MAX_AXON_ROTATION = 180.98;
    private final double MAX_GOBILDA_ROTATION = 300;

    public enum SlideState {
        REST,
        INTAKE,
        OUTTAKE,
        HANG
    }

    public enum ArmState {
        REST,
        LEFT,
        RIGHT,
        INTAKE,
        HANG
    }

    public enum SpecimenState {
        INTAKE,
        OUTTAKE, // The state right before scoring
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
        PICKUP,
        LOW,
        DOWN
    }

    public enum HangState {
        REST,
        HANGING
    }

    public enum SampleState {
        NONE,
        RED,
        YELLOW,
        BLUE
    }

    SlideState slideState;
    ArmState armState;
    SpecimenState specimenState;
    IntakeState intakeState;
    WristState wristState;
    HangState hangState;
    SampleState sampleState;

    public int targetSlidePosition;

    // Constants, needs to be adjusted based on testing
    // Linear Slides
    protected final int REST_POSITION_SLIDES = 0;
    protected final int  INTAKE_POSITION_SLIDES = 500;
    protected final int OUTTAKE_POSITION_SLIDES = 2500;
    protected final int MAX_EXTEND_POSITION = 3000;
    protected final int MANUAL_INCREMENT = 40;
    protected final double DEFAULT_SLIDE_POWER = 1;
    // Hanging
    protected final int ASCENT_LV2_READY = 1500;
    protected final int PRE_ASCENT_LV2 = 1100;
    protected final int ASCENT_LV2 = 400;


    private final double MAX_INTAKE_WRIST_ROTATION = 236.0; // The new neutral. 12/7/24

    protected final double WRIST_NEUTRAL = 0.5;
    protected final double WRIST_UP = WRIST_NEUTRAL + 120.0/MAX_INTAKE_WRIST_ROTATION;
    protected final double WRIST_DROPOFF = WRIST_NEUTRAL - 30.0/MAX_INTAKE_WRIST_ROTATION;
    protected final double WRIST_LOW = WRIST_NEUTRAL - 54.0/MAX_INTAKE_WRIST_ROTATION;
    protected final double WRIST_PICKUP = WRIST_NEUTRAL - 30.0/MAX_INTAKE_WRIST_ROTATION;
    protected final double WRIST_DOWN = WRIST_NEUTRAL - 90.0/MAX_INTAKE_WRIST_ROTATION;

    // Coaxial V4B positions
    // Lower servos. Axon, standard rotation of 180.98 degrees.
    protected final double MAX_LOWER_BAR_ROTATION = 200.5; // 144/255
    protected final double V4B_LOWER_CENTER = 0.5;
    protected final double V4B_LOWER_REST = V4B_LOWER_CENTER + 45.0/MAX_LOWER_BAR_ROTATION;
    protected final double V4B_LOWER_LEFT = V4B_LOWER_CENTER - 90.0/MAX_LOWER_BAR_ROTATION;
    protected final double V4B_LOWER_RIGHT = V4B_LOWER_CENTER + 90.0/MAX_LOWER_BAR_ROTATION;
    protected final double V4B_LOWER_INITIAL = V4B_LOWER_CENTER + 70.0/MAX_LOWER_BAR_ROTATION;
    protected final double V4B_LOWER_LEFT_AUTO = V4B_LOWER_CENTER - 40.0/MAX_LOWER_BAR_ROTATION;
    protected final double V4B_LOWER_RIGHT_AUTO = V4B_LOWER_CENTER + 40.0/MAX_LOWER_BAR_ROTATION;

    // Upper servo. Axon, standard rotation of 180.98 degrees.
    private final double MAX_UPPER_BAR_ROTATION = 236.7; // 170/255
    protected final double V4B_UPPER_CENTER = 0.5;
    protected final double V4B_UPPER_LEFT = V4B_UPPER_CENTER - 100.0/MAX_UPPER_BAR_ROTATION;
    protected final double V4B_UPPER_INITIAL = V4B_UPPER_CENTER - 80.0/MAX_UPPER_BAR_ROTATION;
    protected final double V4B_UPPER_RIGHT = V4B_UPPER_CENTER + 100.0/MAX_UPPER_BAR_ROTATION;
    protected final double UPPER_ALT_INTAKE_ANGLE = 40.0/MAX_UPPER_BAR_ROTATION;

    // 0 is lower servo position. 1 is upper servo position.
    protected final double[] ARM_LEFT_POS = {V4B_LOWER_CENTER, V4B_UPPER_LEFT, WRIST_NEUTRAL};
    protected final double[] ARM_LEFT_POS_AUTO = {V4B_LOWER_LEFT_AUTO, 0.5-UPPER_ALT_INTAKE_ANGLE, WRIST_NEUTRAL};
//    protected final double[] ARM_RIGHT_POS_AUTO = {V4B_LOWER_RIGHT_AUTO, 0.5+UPPER_ALT_INTAKE_ANGLE, WRIST_NEUTRAL};
//    protected final double[] ARM_RIGHT_POS = {V4B_LOWER_CENTER, V4B_UPPER_RIGHT, WRIST_NEUTRAL};
    protected final double[] ARM_REST_POS = {V4B_LOWER_REST, V4B_UPPER_LEFT, WRIST_NEUTRAL};
    protected final double[] ARM_INTAKE_POS = {V4B_LOWER_CENTER, V4B_UPPER_CENTER, WRIST_PICKUP};
    protected final double[] MEGA_REST_POS = {V4B_LOWER_CENTER, V4B_UPPER_CENTER, WRIST_NEUTRAL};

    // Specimen Actuator Positions.
    private final double MAX_SPECIMEN_BAR_ROTATION = 355;
    private final double MAX_SPECIMEN_WRIST_ROTATION = MAX_GOBILDA_ROTATION;

    protected final double SPECIMEN_BAR_NEUTRAL = 0.5;
    protected final double SPECIMEN_BAR_INITIAL_ANGLE = SPECIMEN_BAR_NEUTRAL +(69.0+90.0)/MAX_SPECIMEN_BAR_ROTATION;
    protected final double SPECIMEN_BAR_INTAKE_ANGLE = SPECIMEN_BAR_NEUTRAL +(69.0+90.0)/MAX_SPECIMEN_BAR_ROTATION;
    protected final double SPECIMEN_BAR_OUTTAKE_ANGLE = SPECIMEN_BAR_NEUTRAL -55.0/MAX_SPECIMEN_BAR_ROTATION;
    protected final double SPECIMEN_BAR_STOP_ANGLE = SPECIMEN_BAR_NEUTRAL +10.0/MAX_SPECIMEN_BAR_ROTATION;

    protected final double SPECIMEN_WRIST_NEUTRAL = 0.5;
    protected final double SPECIMEN_WRIST_INITIAL_ANGLE = SPECIMEN_WRIST_NEUTRAL -(90.0+30.0)/MAX_SPECIMEN_WRIST_ROTATION;
    protected final double SPECIMEN_WRIST_INTAKE_ANGLE = SPECIMEN_WRIST_NEUTRAL -60.0/MAX_SPECIMEN_WRIST_ROTATION;
    protected final double SPECIMEN_WRIST_OUTTAKE_ANGLE = SPECIMEN_WRIST_NEUTRAL +1.0/MAX_SPECIMEN_WRIST_ROTATION;
    protected final double SPECIMEN_WRIST_TRANSITION_OFF = SPECIMEN_WRIST_NEUTRAL -120.0/MAX_SPECIMEN_WRIST_ROTATION;

    // Worm Gear
    protected final int HANG_WORM_READY = 500;
    protected final int HANG_WORM_LV2 = -420;

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
    public LED red;
    public LED green;

    protected boolean redSide = false;
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
            specimenState = SpecimenState.INTAKE;
            intakeState = IntakeState.IDLE;
            wristState = WristState.NEUTRAL;
            hangState = HangState.REST;

            targetSlidePosition = REST_POSITION_SLIDES;
            intakeWrist.setPosition(WRIST_UP);
            setV4BPosition(V4B_LOWER_INITIAL, V4B_UPPER_INITIAL);
            specimenBar.setPosition(SPECIMEN_BAR_INITIAL_ANGLE);
            specimenWrist.setPosition(SPECIMEN_WRIST_INITIAL_ANGLE);
        }

        sampleState = SampleState.NONE;
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
        setV4BPosition(V4B_LOWER_INITIAL, V4B_UPPER_INITIAL);
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

    /** @noinspection ArraysAsListWithZeroOrOneArgument*/
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
    }

    public void initSensors(HardwareMap hardwareMap, boolean isRedAlliance) {
        racist = hardwareMap.get(ColorSensor.class, "racist");
        redSide = isRedAlliance;
        slideSwitch = hardwareMap.get(TouchSensor.class, "slide_limit");
        red = hardwareMap.get(LED.class, "red");
        green = hardwareMap.get(LED.class, "green");

        green.off();
        red.off();
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
    ElapsedTime stallTimer = new ElapsedTime();
    boolean resetting = false;
    public void resetSlideEncoders() {
        for (DcMotorEx m : slideMotors) {
            m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        resetting = false;
    }
    // Method to simplify getting the current linear slides position
    public int getSlidesPosition() {
        return slideMotors.get(0).getCurrentPosition();
    }

    public void setSlideState(SlideState state, boolean reset) {
        resetting = reset;
        stallTimer.reset();
        slideState = state;
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

    // Delay time must be specified in the state loop
    ElapsedTime armTimer = new ElapsedTime();
    public void setArmState(ArmState state, boolean delayed) {
        if (delayed) {
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

    public String getColorState() {
        return sampleState.name();
    }

    // Color Sensing
    public boolean withinYellowRange() {
        return racist.green() > racist.red() && racist.red() > racist.blue() && racist.green() > racist.blue() * 2.9;
    }
    public boolean withinRedRange() {
        return racist.red() > racist.green() && racist.green() > racist.blue() && racist.red() > racist.blue() * 2.9;
    }
    public boolean withinBlueRange() {
        return racist.blue() > racist.red() && racist.green() > racist.red() && racist.blue() > racist.red() * 2.9;
    }
    private int prevRed = 0;
    private int prevGreen = 0;
    private int prevBlue = 0;
    private int redDelta = 0;
    private int greenDelta = 0;
    private int blueDelta = 0;
    final int DELTA_THRESHOLD = 100;
    public boolean getSampleDetected() {
        return (redDelta > DELTA_THRESHOLD) || (greenDelta > DELTA_THRESHOLD) || (blueDelta > DELTA_THRESHOLD);
    }
    public void getColorDetections() {
        redDelta = Math.abs(racist.red() - prevRed);
        greenDelta = Math.abs(racist.green() - prevGreen);
        blueDelta = Math.abs(racist.blue() - prevBlue);

        prevRed = racist.red();
        prevGreen = racist.green();
        prevBlue = racist.blue();
    }
    public void switchColorState() {
        if (withinYellowRange()) {
            sampleState = SampleState.YELLOW;
        } else if (withinRedRange()) {
            sampleState = SampleState.RED;
        } else if (withinBlueRange()) {
            sampleState = SampleState.BLUE;
        } else {
            sampleState = SampleState.NONE;
        }
    }

    // RGB for yellow is (255, 255, 0)
    public String slideDisplayText = "WEEWOOWEEWOOWEEWOOWEEWOOWEEEEEEEEEEEEEEEEEEEEEEE";
    public String armDisplayText = "HEEEEEEEEEEEEEEEEEEEEYAWxd";
    public String intakeDisplayText = "NOMMMMMMMMMMMMMMMMM";
    public String wristDisplayText = "YEEEEEEEEEEEEEEET";
    public String hangDisplayText = "RRRRRRRRRRRRRRRRe";
}
