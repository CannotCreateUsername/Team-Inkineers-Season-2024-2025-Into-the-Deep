package org.firstinspires.ftc.teamcode.robot.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

public class ArmSubsystemAuto extends ArmSubsystem {

    @Override
    void init(HardwareMap hardwareMap, boolean isRedAlliance) {
        redSide = isRedAlliance;
        // Map the actuators
        slideMotors = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, "slide1"),
                hardwareMap.get(DcMotorEx.class, "slide2")
        );
        intake = hardwareMap.get(CRServo.class, "intake");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
        wrist = hardwareMap.get(Servo.class, "wrist");

        // Set Motor Modes & Directions
        for (DcMotorEx m : slideMotors) {
            m.setDirection(DcMotorSimple.Direction.REVERSE);
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

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
    public boolean finished = false;

    public Action controlSlides() {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    targetSlidePosition = OUTTAKE_POSITION_SLIDES;
                    set = true;
                }

                runSlideMotorsPID(0.5);
                return !finished;
            }
        };
    }

    public Action controlWrist() {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    wristState = WristState.NEUTRAL;
                    set = true;
                }
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
                }
                return finished;
            }
        };
    }

    public Action score() {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Initialized
                if (!set) {
                    targetSlidePosition = INTAKE_POSITION_SLIDES;
                    set = true;
                }

                return Math.abs(error) > 20;
            }
        };
    }

    public Action reset() {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Initialized
                if (!set) {
                    autoTimer.reset();
                    targetSlidePosition = REST_POSITION_SLIDES;
                    wristState = WristState.UP;
                    set = true;
                }

                intake.setPower(-.1);
                return autoTimer.seconds() < 2;
            }
        };
    }

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

    public Action spinOut() {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    autoTimer.reset();
                    set = true;
                }

                intake.setPower(-1);
                return autoTimer.seconds() < 0.8;
            }
        };
    }
}
