package org.firstinspires.ftc.teamcode.robot.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSubsystemAuto extends ArmSubsystem {

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
                        setV4BPosition(ARM_INTAKE_POS);
                        break;
                    case RIGHT:
                        setV4BPosition(ARM_RIGHT_POS_AUTO);
                        break;
                }

                // Control specimen arm
                switch(specimenState) {
                    case HANG:
                        specimenWrist.setPosition(SPECIMEN_WRIST_INITIAL_ANGLE);
                        break;
                    case INTAKE:
                        if (armState != ArmState.RIGHT) {
                            specimenWrist.setPosition(SPECIMEN_WRIST_INTAKE_ANGLE);
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
                moveV4B(ArmState.REST),
                moveWrist(WristState.UP),
                new SequentialAction(
                        moveSpecimenClaw(SpecimenClawState.CLOSED),
                        new SleepAction(0.2),
                        moveSpecimen(SpecimenState.OUTTAKE)
                )
        );
    }

    public Action hangSpecimenTransition(Action driveAction, boolean ending) {
        return new SequentialAction(
                moveSpecimen(SpecimenState.TRANSITION),
                new SleepAction(0.2),
                ending ? readyEnd() : readyIntake(),
                driveAction
        );
    }
    public Action pickUpAndDropOff() {
        return new SequentialAction(
                pickUpSample(),
                new SleepAction(0.4),
                dropOffSample()
        );
    }

    public Action pickUpSample() {
        return new SequentialAction(
                new ParallelAction(
                        moveV4B(ArmState.LEFT),
                        moveIntake(IntakeState.IN, 1.3),
                        new SequentialAction(
                                new SleepAction(0.1),
                                moveWrist(WristState.PICKUP)
                        )
                )
        );
    }

    public Action dropOffSample() {
        return new SequentialAction(
                new ParallelAction(
                        moveWrist(WristState.NEUTRAL),
                        moveV4B(ArmState.RIGHT),
                        moveIntake(IntakeState.OUT)
                )
        );
    }
    public Action dropOffSample2() {
        return new SequentialAction(
                new ParallelAction(
                        moveWrist(WristState.NEUTRAL),
                        moveV4B(ArmState.LEFT),
                        moveIntake(IntakeState.OUT)
                )
        );
    }

    public Action readyIntake() {
        return new ParallelAction(
                moveIntake(IntakeState.IDLE),
                moveSpecimenClaw(SpecimenClawState.OPEN),
                moveV4B(ArmState.REST),
                moveWrist(WristState.UP),
                moveSpecimen(SpecimenState.INTAKE)
        );
    }

    public Action readyEnd() {
        return new ParallelAction(
                moveIntake(IntakeState.IDLE),
                moveSpecimenClaw(SpecimenClawState.OPEN),
                moveV4B(ArmState.REST),
                moveWrist(WristState.UP),
                moveSpecimen(SpecimenState.INTAKE)
        );
    }

    // PRIMITIVE ACTIONS
    // PRIMITIVE ACTIONS
    // PRIMITIVE ACTIONS

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

    public Action moveIntake(IntakeState state) {
        return telemetryPacket -> {
            intakeState = state;
            return false;
        };
    }

    ElapsedTime intakeTimer = new ElapsedTime();

    public Action moveIntake(IntakeState state, double seconds) {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!set) {
                    intakeTimer.reset();
                    intakeState = state;
                    set = true;
                }

                if (intakeTimer.seconds() > seconds-0.1)
                    intakeState = IntakeState.IDLE;

                return intakeTimer.seconds() < seconds;
            }
        };
    }

    public Action terminate() {
        return telemetryPacket -> {
            finished = true;
            return false;
        };
    }

    @Deprecated
    public Action setTargetSlidePosition(int position) {
        return telemetryPacket -> {
            targetSlidePosition = position;
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

}
