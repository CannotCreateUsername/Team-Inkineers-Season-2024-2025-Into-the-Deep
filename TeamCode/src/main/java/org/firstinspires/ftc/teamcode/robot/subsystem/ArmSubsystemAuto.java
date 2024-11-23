package org.firstinspires.ftc.teamcode.robot.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSubsystemAuto extends ArmSubsystem {

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
                setIntakePowers(1);
                if (autoTimer.seconds() > time-0.2) {
                    setIntakePowers(0);
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
