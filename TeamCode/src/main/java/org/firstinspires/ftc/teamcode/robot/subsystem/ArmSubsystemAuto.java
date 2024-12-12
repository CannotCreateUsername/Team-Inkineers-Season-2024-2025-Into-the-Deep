package org.firstinspires.ftc.teamcode.robot.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSubsystemAuto extends ArmSubsystem {

    ElapsedTime autoTimer = new ElapsedTime();
    private final int SLIDES_OFFSET = 850;
    public boolean finished = false;
    double autoSlidePow = DEFAULT_SLIDE_POWER;

    public Action controlActuators() {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    finished = false;
                    targetSlidePosition = OUTTAKE_POSITION_SLIDES;
                    wristState = WristState.SCORE;
                    set = true;
                }

                // Control Slides
                runSlideMotorsPID(autoSlidePow);
                // Control Wrist
                switch (wristState) {
                    case NEUTRAL:
                        wrist.setPosition(WRIST_PICKUP);
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
                // V4B Controlled Independently

                return !finished;
            }
        };
    }

    public Action transitionToScore() {
        return new SequentialAction(
                pickUpSpecimen(1),
                resetArm(),
                new SleepAction(0.5), // Wait for reset to complete
                readySpecimen(),
                new SleepAction(0.5) // Wait for reset to complete
        );
    }
    public Action scoreAndTransitionToPickup(boolean done) {
        return new SequentialAction(
                score(),
                resetArm(),
                resetSlides(done)
        );
    }
    public Action pickUpAndDropOff() {
        return new SequentialAction(
                pickUpSample(),
                resetArm(),
                new SleepAction(0.5), // Wait for reset to complete
                dropOffSample(),
                resetArm(),
                new SleepAction(0.5) // Wait for reset to complete
        );
    }

    public Action resetArm() {
        return telemetryPacket -> {
            targetSlidePosition = INTAKE_POSITION_SLIDES;
            setArmPosition(ARM_REST_POS, WristState.UP);
            setIntakePowers(0);
            return false;
        };
    }

    public Action readySpecimen() {
        return telemetryPacket -> {
            targetSlidePosition = OUTTAKE_POSITION_SLIDES;
            setArmPosition(ARM_LEFT_POS, WristState.SCORE);
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


    public Action resetSlides(boolean done) {
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

                targetSlidePosition = REST_POSITION_SLIDES;
                if (slideSwitch.isPressed()) {
                    resetSlideEncoders();
                    if (!done)
                        // Return slides to driving height if the autonomous period is not done.
                        // Otherwise, when autonomous period is finished, it will stay at rest.
                        targetSlidePosition = INTAKE_POSITION_SLIDES;
                    // Break out if the magnetic limit switch is activated.
                    return false;
                }

                // Break out if it does not reset within 1.2 seconds.
                return autoTimer.seconds() < 1.2;
            }
        };
    }

    public Action pickUpSample() {
        return new Action() {
            boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    autoTimer.reset();
                    setArmPosition(ARM_LEFT_POS, WristState.DOWN);
                    set = true;
                }

                if (autoTimer.seconds() > 0.5) {
                    targetSlidePosition = INTAKE_POSITION_SLIDES;
                    setIntakePowers(1);
                }
                return autoTimer.seconds() < 1.5;
            }
        };
    }

    public Action dropOffSample() {
        return new Action() {
            boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    autoTimer.reset();
                    setArmPosition(ARM_RIGHT_POS, WristState.DOWN);
                    set = true;
                }

                if (autoTimer.seconds() > 0.5) {
                    setIntakePowers(-0.5);
                }
                return autoTimer.seconds() < 1;
            }
        };
    }

    public Action pickUpSpecimen(double time) { // Runs in parallel with a trajectory action
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    autoTimer.reset();
                    targetSlidePosition = INTAKE_POSITION_SLIDES;
                    setArmPosition(ARM_RIGHT_POS, WristState.NEUTRAL);
                    set = true;
                }
                setIntakePowers(1);

                // turn off intake in a separate action...

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
    public Action moveArm(ArmState state) {
        return telemetryPacket -> {
            armState = state;
            return false;
        };
    }

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
