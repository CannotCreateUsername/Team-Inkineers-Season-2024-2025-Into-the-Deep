package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.left;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right.PPCoords;
import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemAutoPP;

@Autonomous(name = "LEFT side Auto", group = "Autonomous")
public class PPSampleAuto extends OpMode {
    private Telemetry telemetryA;

    private Follower follower;
    private ArmSubsystemAutoPP armSubsystem;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    PPCoords coords = new PPCoords();

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain moveOut;

    Pose start = new Pose(8, 108, coords.STRAIGHT);
    Pose out = new Pose(40, 120);

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        moveOut = follower.pathBuilder()
                // Go to Above Sample 1
                .addPath(new BezierLine(
                        new Point(start),
                        new Point(out)
                ))
                .setConstantHeadingInterpolation(coords.STRAIGHT)
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                /* Run pushing path chain */
                armSubsystem.restArm(true);
                follower.followPath(moveOut);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Control arm subsystem
        armSubsystem.controlSpecimenArm();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
        follower.telemetryDebug(telemetryA);
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(start);
        buildPaths();

        // Robot Systems
        armSubsystem = new ArmSubsystemAutoPP();
        armSubsystem.init(hardwareMap, false, true);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This opmode has a serious case of inconsistency. Press play and PRAY");
        telemetryA.update();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

