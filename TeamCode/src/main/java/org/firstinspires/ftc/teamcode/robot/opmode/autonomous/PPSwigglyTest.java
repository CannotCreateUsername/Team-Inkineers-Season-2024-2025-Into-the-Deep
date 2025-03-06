package org.firstinspires.ftc.teamcode.robot.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
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

@Autonomous(name = "PP Swiggly Test", group = "Autonomous")
public class PPSwigglyTest extends OpMode {
    private Telemetry telemetryA;

    private Follower follower;
    private ArmSubsystemAutoPP armSubsystem;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    PPCoords coords = new PPCoords();

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scoreSpecimen, pickUpSpecimen;
    private PathChain pushSample1, pushSample2, specimenRebound;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        pushSample1 = follower.pathBuilder()
                // Go to Above Sample 1
                .addPath(new BezierCurve(
                        new Point(coords.startPose),
                        new Point(coords.controlPush1),
                        new Point(coords.controlPush12),
                        new Point(coords.push1Pose)
                ))
                .setConstantHeadingInterpolation(coords.STRAIGHT)
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        pushSample2 = follower.pathBuilder()
                // Go to Above Sample 1
                .addPath(new BezierCurve(
                        new Point(coords.push1Pose),
                        new Point(coords.controlPush12),
                        new Point(coords.controlPush1),
                        new Point(coords.startPose)
                ))
                .setConstantHeadingInterpolation(coords.STRAIGHT)
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        // RESET COORDS HERE
        specimenRebound = follower.pathBuilder()
                // Position robot for specimen pickup
                .addPath(new BezierCurve(
                        new Point(coords.observationPose3),
                        new Point(coords.controlSpecimen0),
                        new Point(coords.pickupSpecimenPose)
                ))
                .setLinearHeadingInterpolation(coords.STRAIGHT, coords.ROTATED)
                .addPath(new BezierCurve(
                        new Point(coords.pickupSpecimenPose),
                        new Point(coords.controlSpecimen0),
                        new Point(coords.observationPose3)
                ))
                .setLinearHeadingInterpolation(coords.ROTATED, coords.STRAIGHT)
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */

    private int cycles = 0;
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                armSubsystem.restArm(false);
                /* Run pushing path chain */
                if (!follower.isBusy()) {
                    if (cycles < 4) {
                        follower.followPath(pushSample1);
                        setPathState(1);

                        cycles++;
                    } else {
                        setPathState(-1);
                    }
                }
                break;
            case 1:
//                // Create new paths to avoid scoring in the same place
//                // From Pickup to Score
//                Point newScore = new Point(coords.scorePose0.getX(), coords.scorePose0.getY()+ 2*cycles);
//                scoreSpecimen = new Path(new BezierCurve(
//                        new Point(coords.pickupSpecimenPose),
//                        new Point(40, 30),
//                        new Point(30, 62),
//                        newScore
//                ));
//                scoreSpecimen.setConstantHeadingInterpolation(coords.ROTATED);
//                scoreSpecimen.setZeroPowerAccelerationMultiplier(4);
//                // From Score to Pickup
//                pickUpSpecimen = new Path(new BezierCurve(
//                        newScore,
//                        new Point(30, 62),
//                        new Point(40, 27),
//                        new Point(coords.pickupSpecimenPose)
//                ));
//                pickUpSpecimen.setConstantHeadingInterpolation(coords.ROTATED);
//                pickUpSpecimen.setZeroPowerAccelerationMultiplier(2);
//
//                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
//                    follower.followPath(scoreSpecimen,true);
//                    setPathState(6);
//                }
                if (!follower.isBusy()) {
                    follower.followPath(pushSample2);
                    setPathState(0);
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
        follower.setStartingPose(coords.startPose);
        buildPaths();

        // Robot Systems
        armSubsystem = new ArmSubsystemAutoPP();
        armSubsystem.init(hardwareMap, false, true);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This opmode has a serious case of LIGMA. Ong, no cap.");
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

