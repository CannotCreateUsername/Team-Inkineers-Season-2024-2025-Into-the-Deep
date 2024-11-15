package org.firstinspires.ftc.teamcode.robot.subsystem;

import static org.firstinspires.ftc.teamcode.robot.constants.PIDConstants.kPslides;
import static org.firstinspires.ftc.teamcode.robot.constants.PIDConstants.threshold_slides;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/** @noinspection FieldCanBeLocal*/
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

    // Needs to be adjusted based on testing
    // Linear Slides
    final int REST_POSITION_SLIDES = 0;
    final int INTAKE_POSITION_SLIDES = 380;
    final int OUTTAKE_POSITION_SLIDES = 2200;
    final int MAX_EXTEND_POSITION = 3500;
    final int MANUAL_INCREMENT = 20;

    // Max Rotation for 2000-0025-0002 Torque Servo: 300 degrees
    // 90 degrees is position +- 90/300
    final double WRIST_NEUTRAL = 0.5;
    final double WRIST_UP = WRIST_NEUTRAL+90.0/300;
    final double WRIST_DOWN = WRIST_NEUTRAL-72.0/300;
    final double WRIST_SCORE = WRIST_NEUTRAL-20.0/300;

    // Linear Actuator
    final int HANG_UP = 3050;
    final int HANG_DOWN = 1100;
    final int HANG_REST = 0;

    // Declare actuator variables
    public List<DcMotorEx> slideMotors; // Initialize as list to support potential multiple motors

    public Servo wrist;
    public CRServo intake;
    public CRServo intake2;

    public DcMotor hangMotor;
    public Servo hangServo;

    ColorSensor racist;

    boolean redSide = false;
    abstract public void init(HardwareMap hardwareMap, boolean isRedAlliance);

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
