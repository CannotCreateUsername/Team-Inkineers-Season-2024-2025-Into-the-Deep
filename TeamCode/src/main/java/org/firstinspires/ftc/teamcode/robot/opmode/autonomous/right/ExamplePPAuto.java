package org.firstinspires.ftc.teamcode.robot.opmode.autonomous.right;

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

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.subsystem.ArmSubsystemAutoPP;

@Autonomous(name = "PP Auto", group = "Autonomous")
public class ExamplePPAuto extends OpMode {

    private Follower follower;
    private ArmSubsystemAutoPP armSubsystem;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    RightAutoCoordsPP coords = new RightAutoCoordsPP();

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park;
    private PathChain pushSample1, backSample1, pushSample2, backSample2, pushSample3, backSample3;
    private PathChain pickUpSpecimen0, scoreSpecimen, pickUpSpecimen;

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

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pushSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(coords.startPose), new Point(coords.push1Pose)))
                .setConstantHeadingInterpolation(coords.ROTATED)
                .build();

        backSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(coords.push1Pose), new Point(coords.observationPos1)))
                .setConstantHeadingInterpolation(coords.ROTATED)
                .build();

        backSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(coords.observationPos1), new Point(coords.push2Pose)))
                .setConstantHeadingInterpolation(coords.ROTATED)
                .build();

        pushSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(coords.push2Pose), new Point(coords.observationPos2)))
                .setConstantHeadingInterpolation(coords.ROTATED)
                .build();

        pushSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(coords.observationPos2), new Point(coords.push3Pose)))
                .setConstantHeadingInterpolation(coords.ROTATED)
                .build();

        backSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(coords.push3Pose), new Point(coords.observationPos3)))
                .setConstantHeadingInterpolation(coords.ROTATED)
                .build();

        // Do something weird here. IDK.
        pickUpSpecimen0 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(coords.observationPos3), new Point(coords.specimenPickupPose)))
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
//        park = new Path(new BezierCurve(new Point(coords.scorePose), /* Control Point */ new Point(coords.parkControlPose), new Point(coords.parkPose)));
//        park.setLinearHeadingInterpolation(coords.scorePose.getHeading(), coords.parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Run to Push 1 Pose
                follower.followPath(pushSample1);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close
                (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    // Run to Observation Pose 1

                    follower.followPath(backSample1,true);
                    setPathState(2);
                }
                break;
            case 2:

                if(!follower.isBusy()) {
                    // Run to Push 2 Pose

                    follower.followPath(pushSample2,true);
                    setPathState(3);
                }
                break;
            case 3:

                if(!follower.isBusy()) {
                    // Run to Observation Pose 2

                    follower.followPath(backSample2,true);
                    setPathState(4);
                }
                break;
            case 4:

                if(!follower.isBusy()) {
                    // Run to Push 3 Pose

                    follower.followPath(pushSample3,true);
                    setPathState(5);
                }
                break;
            case 5:

                if(!follower.isBusy()) {
                    // Run to Observation Pose 3

                    follower.followPath(backSample3,true);
                    setPathState(6);
                }
                break;
            case 6:

                if(!follower.isBusy()) {
                    // Run to Pick Up Specimen Pose

                    follower.followPath(backSample3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    // Run to Score Pose

                    follower.followPath(park,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
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

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
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

