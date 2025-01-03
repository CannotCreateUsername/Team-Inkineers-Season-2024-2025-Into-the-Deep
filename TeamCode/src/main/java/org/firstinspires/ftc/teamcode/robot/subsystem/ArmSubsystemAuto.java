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
                    case LEFT:
                        setV4BPosition(ARM_LEFT_POS);
                        break;
                    case RIGHT:
                        if (specimenState != SpecimenState.INTAKE) {
                            setV4BPosition(ARM_RIGHT_POS);
                            break;
                        }
                }

                // Control specimen arm
                switch(specimenState) {
                    case INTAKE:
                        if (armState != ArmState.RIGHT) {
                            specimenBar.setPosition(SPECIMEN_BAR_INTAKE_ANGLE);
                            if (specimenTimer.seconds() > 0.5) {
                                specimenWrist.setPosition(SPECIMEN_WRIST_TRANSITION_OFF);
                            } else {
                                specimenWrist.setPosition(SPECIMEN_WRIST_INTAKE_ANGLE);
                            }
                        }
                        break;
                    case AUTO:
                        specimenBar.setPosition(SPECIMEN_BAR_STRAIGHT_ANGLE);
                        specimenWrist.setPosition(SPECIMEN_WRIST_TRANSITION_ON);
                        break;
                    case OUTTAKE:
                        specimenBar.setPosition(SPECIMEN_BAR_OUTTAKE_ANGLE);
                        specimenWrist.setPosition(SPECIMEN_WRIST_TRANSITION_ON);
                        break;
                    case HANG:
                        specimenBar.setPosition(SPECIMEN_BAR_NEUTRAL);
                        specimenWrist.setPosition(SPECIMEN_WRIST_NEUTRAL);
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
                    case SCORE:
                        intakeWrist.setPosition(WRIST_SCORE);
                        break;
                    case LOW:
                        intakeWrist.setPosition(WRIST_PICKUP_LOW);
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
                moveV4B(ArmState.REST),
                moveWrist(WristState.NEUTRAL),
                moveSpecimen(SpecimenState.AUTO)
        );
    }

    public Action hangSpecimenTransition(Action driveAction) {
        return new SequentialAction(
                moveSpecimen(SpecimenState.OUTTAKE),
                new SleepAction(0.3),
                new ParallelAction(
                        driveAction,
                        readyIntake()
                )
        );
    }

    public Action pickUpSample() {
        return new SequentialAction(
                new ParallelAction(
                        moveV4B(ArmState.LEFT),
                        moveWrist(WristState.LOW),
                        moveSpecimen(SpecimenState.HANG)
                ),
                new SleepAction(0.5),
                moveIntake(IntakeState.IN, 0.4),
                moveWrist(WristState.NEUTRAL)
        );
    }

    public Action dropOffSample() {
        return new SequentialAction(
                new ParallelAction(
                        moveV4B(ArmState.RIGHT),
                        moveSpecimen(SpecimenState.HANG)
                ),
                new SleepAction(0.5),
                new ParallelAction(
                        moveWrist(WristState.DROPOFF),
                        moveIntake(IntakeState.OUT, 0.2)
                )
        );
    }

    public Action readyIntake() {
        return new ParallelAction(
                moveV4B(ArmState.REST),
                moveWrist(WristState.NEUTRAL),
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

    @Deprecated
    public Action resetSlidesPosition() {
        return telemetryPacket -> {
            resetSlideEncoders();
            return false;
        };
    }

    public Action moveV4B(ArmState state) {
        return telemetryPacket -> {
            setArmState(state);
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
