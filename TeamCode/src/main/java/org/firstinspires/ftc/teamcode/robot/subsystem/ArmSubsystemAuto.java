package org.firstinspires.ftc.teamcode.robot.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
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
                    wristState = WristState.SCORE;
                    set = true;
                }

                // Control Slides
                runSlideMotorsPID(autoSlidePow);
                // Control Wrist
                switch (wristState) {
                    case NEUTRAL:
                        if (wristTimer.seconds() > 0.5)
                            wrist.setPosition(WRIST_PICKUP);
                        break;
                    case UP:
                        wrist.setPosition(WRIST_UP);
                        break;
                    case DOWN:
                        if (wristTimer.seconds() > 0.5)
                            wrist.setPosition(WRIST_DOWN);
                        break;
                    case SCORE:
                        wrist.setPosition(WRIST_SCORE);
                        break;
                    case LOW:
                        wrist.setPosition(WRIST_DROPOFF);
                        break;
                }
                // V4B Controlled Independently

                return !finished;
            }
        };
    }

    public Action transitionToScore(Action driveAction) {
        return new SequentialAction(
                new ParallelAction(
                        pickUpSpecimen(1),
                        driveAction
                ),
                resetArm(),
                readySpecimen2()
        );
    }
    public Action scoreAndTransitionToPickup(boolean done) {
        return new SequentialAction(
                score(),
                new ParallelAction(
                        resetArm(),
                        resetSlides(done)
                )
        );
    }
    public Action pickUpAndDropOff() {
        return new SequentialAction(
                pickUpSample(),
                resetArm(),
                dropOffSample(),
                resetArm()
        );
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
            boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    autoTimer.reset();
                    targetSlidePosition = OUTTAKE_POSITION_SLIDES;
                    armState = ArmState.LEFT_FAR;
                    setV4BPosition(V4B_LOWER_LEFT, V4B_UPPER_TRANSITION);
                    set = true;
                }
                if (autoTimer.seconds() > 0.3) {
                    setV4BPosition(ARM_LEFT_POS);
                    setWristState(WristState.SCORE, false);
                }
                return autoTimer.seconds() < 1.5;
            }
        };
    }

    public Action readySpecimen2() {
        return new Action() {
            boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    autoTimer.reset();
                    targetSlidePosition = OUTTAKE_POSITION_SLIDES-50;
                    armState = ArmState.LEFT_FAR;
                    set = true;
                }
                if (autoTimer.seconds() > 0.6) {
                    setV4BPosition(V4B_LOWER_LEFT, V4B_UPPER_TRANSITION);
                }
                if (autoTimer.seconds() > 1) {
                    setV4BPosition(ARM_LEFT_POS);
                    setWristState(WristState.SCORE, false);
                }
                return autoTimer.seconds() < 1.6;
            }
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
                    targetSlidePosition = INTAKE_SPECIMEN_POSITION_SLIDES +SLIDES_OFFSET;
                    set = true;
                }
                setIntakePowers(0.2);
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
                        targetSlidePosition = INTAKE_SPECIMEN_POSITION_SLIDES;
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
                    targetSlidePosition = INTAKE_SPECIMEN_POSITION_SLIDES;
                    setV4BPosition(V4B_LOWER_LEFT, V4B_UPPER_TRANSITION);
                    setWristState(WristState.DOWN, true);
                    set = true;
                }
                if (autoTimer.seconds() > 0.4) {
                    setV4BPosition(ARM_LEFT_POS);
                }

                if (autoTimer.seconds() > 0.8) {
                    targetSlidePosition = REST_POSITION_SLIDES;
                    setIntakePowers(1);
                }
                return autoTimer.seconds() < 1.2;
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
                    setV4BPosition(ARM_RIGHT_POS);
                    setWristState(WristState.LOW, false);
                    armState = ArmState.RIGHT_FAR;
                    set = true;
                }

                if (autoTimer.seconds() > 0.6) {
                    setIntakePowers(-0.2);
                }
                return autoTimer.seconds() < 0.9;
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
                    targetSlidePosition = INTAKE_SPECIMEN_POSITION_SLIDES;
                    setV4BPosition(ARM_RIGHT_POS);
                    setWristState(WristState.NEUTRAL, false);
                    set = true;
                }
                setIntakePowers(1);

                // turn off intake in a separate action...

                return autoTimer.seconds() < time;
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
