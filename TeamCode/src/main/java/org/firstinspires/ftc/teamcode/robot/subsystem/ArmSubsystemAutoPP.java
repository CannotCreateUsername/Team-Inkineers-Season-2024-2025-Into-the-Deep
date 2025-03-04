package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSubsystemAutoPP extends ArmSubsystem {


    // TODO: Raise Specimen Arm while pushing to prevent damage against wall


    private int specimenArmState = 0; // 0 intake position. 1 outtake position. 2 transition position.
    private final ElapsedTime specimenSystemTimer = new ElapsedTime();
    public void setSpecimenArmState(int i) {
        specimenArmState = i;
        specimenSystemTimer.reset();
    }

    public void pullUpWrist() {
        setV4BPosition(ARM_REST_POS);
        intakeWrist.setPosition(WRIST_UP);
    }

    public void controlSpecimenArm() {
        /* Timer conditions handle post initialization movements.
        else {} statements handle the default (initial) positions */
        switch (specimenArmState) {
            case 0:
                // INTAKE POSITION
                // 1. Return Bar and Wrist to Pickup
                // 2. Open Claw
                specimenClaw.setPosition(SPECIMEN_CLAW_OPEN);
                specimenBar.setPosition(SPECIMEN_BAR_INTAKE_ANGLE);
                specimenWrist.setPosition(SPECIMEN_WRIST_INTAKE_ANGLE);
                break;
            case 1:
                // OUTTAKE POSITION
                // 1. Close Claw
                // 2. Ready Bar and Wrist to Score after 0.2 seconds
                specimenClaw.setPosition(SPECIMEN_CLAW_CLOSED);
                if (specimenSystemTimer.seconds() > 0.2) {
                    specimenBar.setPosition(SPECIMEN_BAR_OUTTAKE_ANGLE);
                    if (specimenSystemTimer.seconds() > 0.4) {
                        specimenWrist.setPosition(SPECIMEN_WRIST_TRANSITION_OFF);
                    } else {
                        specimenWrist.setPosition(SPECIMEN_WRIST_OUTTAKE_ANGLE);
                    }
                }
                break;
            case 2:
                // TRANSITION POSITION
                // Move Bar and Wrist to Transition
                // Open Claw and Release after 0.2 seconds
                specimenBar.setPosition(SPECIMEN_BAR_TRANSITION_ANGLE);
                specimenWrist.setPosition(SPECIMEN_WRIST_TRANSITION_OFF);
                if (specimenSystemTimer.seconds() > 0.2) {
                    specimenClaw.setPosition(SPECIMEN_CLAW_OPEN);
                    setSpecimenArmState(0);
                }
                break;
            case 3:
                // INITIAL POSITION
                specimenBar.setPosition(SPECIMEN_BAR_INITIAL_ANGLE);
                specimenWrist.setPosition(SPECIMEN_WRIST_INITIAL_ANGLE);
                specimenClaw.setPosition(SPECIMEN_CLAW_CLOSED);
                break;
        }
    }
}
