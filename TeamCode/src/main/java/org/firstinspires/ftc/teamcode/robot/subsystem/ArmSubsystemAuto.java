package org.firstinspires.ftc.teamcode.robot.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

public class ArmSubsystemAuto extends ArmSubsystem {

    @Override
    public void init(HardwareMap hardwareMap, boolean isRedAlliance) {
        redSide = isRedAlliance;
        // Map the actuators
        slideMotors = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, "belt_slide"),
                hardwareMap.get(DcMotorEx.class, "non_slide")
        );
        intake = hardwareMap.get(CRServo.class, "right");
        intake2 = hardwareMap.get(CRServo.class, "left");
        wrist = hardwareMap.get(Servo.class, "wrist");
        racist = hardwareMap.get(ColorSensor.class, "racist");

        // Set Motor Modes
        for (DcMotorEx m : slideMotors) {
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        // Reverse Encoder Motor
        slideMotors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);

        // Initialize Positions; Start at REST
        armState = ArmState.REST;
        intakeState = IntakeState.IDLE;
        wristState = WristState.UP;

        targetSlidePosition = REST_POSITION_SLIDES;
        wrist.setPosition(WRIST_UP);
    }

    ElapsedTime autoTimer = new ElapsedTime();
    private final int SLIDES_OFFSET = 850;
    public boolean finished = false;
    double autoSlidePow = 0.4;

    public Action controlActuators() {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    finished = false;
                    targetSlidePosition = OUTTAKE_POSITION_SLIDES+100;
                    wristState = WristState.SCORE;
                    set = true;
                }

                runSlideMotorsPID(autoSlidePow);
                switch (wristState) {
                    case NEUTRAL:
                        wrist.setPosition(WRIST_NEUTRAL);
                        break;
                    case UP:
                        wrist.setPosition(WRIST_UP);
                        break;
                    case DOWN:
                        wrist.setPosition(WRIST_DOWN);
                        break;
                    case SCORE:
                        wrist.setPosition(WRIST_SCORE);
                        break;
                }
                return !finished;
            }
        };
    }

    public Action readySpecimen() {
        return telemetryPacket -> {
            targetSlidePosition = OUTTAKE_POSITION_SLIDES;
            wristState = WristState.SCORE;
            return false;
        };
    }

    public Action score() {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Initialize
                if (!set) {
                    autoTimer.reset();
                    targetSlidePosition = INTAKE_POSITION_SLIDES+SLIDES_OFFSET;
                    set = true;
                }
                return autoTimer.seconds() < 0.6;
            }
        };
    }

    public Action slidesReset(boolean finished) {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Initialize
                if (!set) {
                    autoTimer.reset();
                    wristState = WristState.UP;
                    set = true;
                }

                if (autoTimer.seconds() > 0.8)
                    targetSlidePosition = REST_POSITION_SLIDES;
                if (autoTimer.seconds() > 1.8 && !finished) {
                    resetSlideEncoders();
                    targetSlidePosition = INTAKE_POSITION_SLIDES-60;
                }

                return autoTimer.seconds() < 2;
            }
        };
    }

    public Action pickUpSpecimen(double time) {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    autoTimer.reset();
                    targetSlidePosition = INTAKE_POSITION_SLIDES-60;
                    wristState = WristState.NEUTRAL;
                    set = true;
                }
                intake.setPower(1);
                intake2.setPower(1);
                if (autoTimer.seconds() > time-0.2) {
                    intake.setPower(0);
                    intake2.setPower(0);
                }
                return autoTimer.seconds() < time;
            }
        };
    }

    public Action terminate() {
        return telemetryPacket -> {
            finished = true;
            return false;
        };
    }

    // UNUSED ACTIONS

    @Deprecated
    public Action spinIn(double time) {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    autoTimer.reset();
                    targetSlidePosition = REST_POSITION_SLIDES;
                    wristState = WristState.DOWN;
                    set = true;
                }

                intake.setPower(1);
                intake2.setPower(1);
                return autoTimer.seconds() < time;
            }
        };
    }

    @Deprecated
    public Action spinOut(double time) {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    autoTimer.reset();
                    wristState = WristState.NEUTRAL;
                    set = true;
                }

                intake.setPower(-1);
                intake2.setPower(-1);
                return autoTimer.seconds() < time;
            }
        };
    }
}
