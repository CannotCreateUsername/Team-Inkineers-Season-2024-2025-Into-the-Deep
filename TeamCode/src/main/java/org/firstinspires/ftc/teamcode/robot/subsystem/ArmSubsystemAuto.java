package org.firstinspires.ftc.teamcode.robot.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
                    set = true;
                }

                // Control Slides
                runSlideMotorsPID(autoSlidePow);

                // Control V4B
                switch (armState) {
                    case REST:
                        setV4BPosition(V4B_LOWER_INITIAL, V4B_UPPER_INITIAL);
                        break;
                    case LEFT:
                        setV4BPosition(ARM_LEFT_POS_AUTO);
                        break;
                    case RIGHT:
                        setV4BPosition(V4B_LOWER_CENTER, UPPER_ALT_INTAKE_ANGLE + 0.5);
                        break;
                }

                // Control specimen arm
                switch(specimenState) {
                    case INTAKE:
                        if (armState != ArmState.RIGHT) {
                            if (specimenTimer.seconds() < 0.15) {
                                specimenWrist.setPosition(SPECIMEN_WRIST_INTAKE_ANGLE);
                            } else {
                                specimenWrist.setPosition(SPECIMEN_WRIST_OUTTAKE_ANGLE);
                            }
                            specimenBar.setPosition(SPECIMEN_BAR_INTAKE_ANGLE);
                        }
                        break;
                    case OUTTAKE:
                        specimenBar.setPosition(SPECIMEN_BAR_OUTTAKE_ANGLE);
                        if (specimenTimer.seconds() > 0.4) {
                            specimenWrist.setPosition(SPECIMEN_WRIST_TRANSITION_OFF);
                        } else {
                            specimenWrist.setPosition(SPECIMEN_WRIST_OUTTAKE_ANGLE);
                        }
                        break;
                    case TRANSITION:
                        specimenBar.setPosition(SPECIMEN_BAR_TRANSITION_ANGLE);
                        specimenWrist.setPosition(SPECIMEN_WRIST_TRANSITION_OFF);
                        break;
                    case HANG:
                        specimenBar.setPosition(SPECIMEN_BAR_NEUTRAL);
                        specimenWrist.setPosition(SPECIMEN_WRIST_NEUTRAL);
                        break;
                }

                switch(specimenClawState) {
                    case CLOSED:
                        specimenClaw.setPosition(SPECIMEN_CLAW_CLOSED);
                        break;
                    case OPEN:
                        specimenClaw.setPosition(SPECIMEN_CLAW_OPEN);
                        break;
                }

                // Control Wrist
                switch (wristState) {
                    case NEUTRAL:
                        intakeWrist.setPosition(WRIST_NEUTRAL);
                        break;
                    case UP:
                        intakeWrist.setPosition(WRIST_UP);
                        break;
                    case DROPOFF:
                        intakeWrist.setPosition(WRIST_DROPOFF);
                        break;
                    case PICKUP:
                        intakeWrist.setPosition(WRIST_LOW);
                        break;
                }

                switch (intakeState) {
                    case IDLE:
                        setIntakePowers(0);
                        break;
                    case IN:
                        setIntakePowers(1);
                        break;
                    case OUT:
                        setIntakePowers(-1);
                        break;
                }

                return !finished;
            }
        };
    }

    public Action readySpecimen() {
        return new ParallelAction(
                moveSpecimenClaw(SpecimenClawState.CLOSED),
                moveV4B(ArmState.REST),
                moveWrist(WristState.UP),
                new SequentialAction(
                        moveSpecimen(SpecimenState.OUTTAKE)
                )
        );
    }

    public Action hangSpecimenTransition(Action driveAction, boolean pickUp) {
        if (pickUp) {
            return new SequentialAction(
                    readyIntake(),
                    new SleepAction(0.3),
                    new ParallelAction(
                            driveAction,
                            new SequentialAction(
                                    new SleepAction(0.5),
                                    pickUpSample(true)
                            )
                    )
            );
        } else {
            return new SequentialAction(
                    readyIntake(),
                    new SleepAction(0.3),
                    driveAction
            );
        }
    }

    public Action pickUpSample(boolean delay) {
        return new SequentialAction(
                new ParallelAction(
                        moveWrist(delay ? WristState.NEUTRAL : WristState.UP),
                        moveV4B(ArmState.LEFT)
                ),
                new SleepAction(delay ? 1.4 : 0.6),
                moveWrist(WristState.NEUTRAL),
                new ParallelAction(
                        moveIntake(IntakeState.IN, 0.8),
                        new SequentialAction(
                                new SleepAction(0.4), // In contact for 0.8 - 0.2 seconds
                                moveWrist(WristState.PICKUP)
                        )
                ),
                moveWrist(WristState.UP)
        );
    }

    public Action dropOffSample() {
        return new SequentialAction(
                new ParallelAction(
                        moveV4B(ArmState.RIGHT)
                ),
                new SleepAction(1),
                moveWrist(WristState.DROPOFF),
                new SleepAction(0.2),
                moveIntake(IntakeState.OUT, 0.4),
                moveWrist(WristState.UP)
        );
    }

    public Action readyIntake() {
        return new ParallelAction(
                moveV4B(ArmState.REST),
                moveWrist(WristState.UP),
                moveSpecimen(SpecimenState.INTAKE)
        );
    }



    // PRIMITIVE ACTIONS
    @Deprecated
    public Action setTargetSlidePosition(int position) {
        return telemetryPacket -> {
            targetSlidePosition = position;
            return false;
        };
    }

    public Action moveSpecimen(SpecimenState state) {
        return telemetryPacket -> {
            setSpecimenState(state);
            return false;
        };
    }

    public Action moveSpecimenClaw(SpecimenClawState state) {
        return telemetryPacket -> {
            setSpecimenClawState(state);
            return false;
        };
    }

    @Deprecated
    public Action resetSlidesPosition() {
        return telemetryPacket -> {
            resetSlideEncoders();
            return false;
        };
    }

    public Action moveV4B(ArmState state) {
        return telemetryPacket -> {
            setArmState(state, false);
            return false;
        };
    }

    public Action moveWrist(WristState state) {
        return telemetryPacket -> {
            wristState = state;
            return false;
        };
    }

    public Action moveIntake(IntakeState state, double seconds) {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    autoTimer.reset();
                    intakeState = state;
                    set = true;
                }

                if (autoTimer.seconds() > seconds-0.1)
                    intakeState = IntakeState.IDLE;

                return autoTimer.seconds() < seconds;
            }
        };
    }

    public Action terminate() {
        return telemetryPacket -> {
            finished = true;
            return false;
        };
    }

}
