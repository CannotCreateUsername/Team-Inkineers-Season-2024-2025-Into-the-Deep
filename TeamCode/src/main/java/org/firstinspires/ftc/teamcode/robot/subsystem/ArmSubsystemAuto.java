package org.firstinspires.ftc.teamcode.robot.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSubsystemAuto extends ArmSubsystem {

    ElapsedTime autoTimer = new ElapsedTime();
    public boolean finished = false;
    double autoSlidePow = DEFAULT_SLIDE_POWER;

    public Action controlActuators() {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    finished = false;
                    wristState = WristState.SCORE;
                    set = true;
                }

                // Control Slides
                runSlideMotorsPID(autoSlidePow);
                // Control Wrist
                switch (wristState) {
                    case NEUTRAL:
                        if (wristTimer.seconds() > 0.5)
                            intakeWrist.setPosition(WRIST_PICKUP);
                        break;
                    case UP:
                        intakeWrist.setPosition(WRIST_UP);
                        break;
                    case DOWN:
                        if (wristTimer.seconds() > 0.5)
                            intakeWrist.setPosition(WRIST_DOWN);
                        break;
                    case SCORE:
                        intakeWrist.setPosition(WRIST_SCORE);
                        break;
                    case LOW:
                        intakeWrist.setPosition(WRIST_DROPOFF);
                        break;
                }
                // V4B Controlled Independently

                return !finished;
            }
        };
    }

    public Action resetArm() {
        return new Action() {
            boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    autoTimer.reset();
                    resetV4B();
                    set = true;
                }
                setIntakePowers(0);
                if (autoTimer.seconds() > 0.5) {
                    setV4BPosition(ARM_REST_POS);
                }
                return autoTimer.seconds() < 1;
            }
        };
    }

    public Action readySpecimen() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }
        };
    }

    public Action resetSlides(boolean done) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                resetSlideEncoders();
                return false;
            }
        };
    }

    public Action moveArm(ArmState state) {
        return telemetryPacket -> {
            armState = state;
            return false;
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

                setIntakePowers(1);
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

                setIntakePowers(-1);
                return autoTimer.seconds() < time;
            }
        };
    }
}
