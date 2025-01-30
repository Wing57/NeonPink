package pedroPathing.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Specywecy", group = "Examples")
public class Spec extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(10.525, 57.090, 0);

    private final Point preloadDunkSpot = new Point(39, 68, Point.CARTESIAN);
    private final Point firstDunkSpot = new Point(39, 70, Point.CARTESIAN);
    private final Point secondDunkSpot = new Point(39, 72, Point.CARTESIAN);
    private final Point thirdDunkSpot = new Point(39, 74, Point.CARTESIAN);

    private final Point firstFerrySpot = new Point(19.6, 25, Point.CARTESIAN);
    private final Point secondFerrySpot = new Point(19.6, 15, Point.CARTESIAN);

    private final Point firstGrabSpot = new Point(14, 19, Point.CARTESIAN);
    private final Point generalGrabSpot = new Point(14, 31, Point.CARTESIAN);


    private Path park;
    private PathChain scorePreload, ferryFirst, ferrySecond, grabFirst, grabSecond, grabThird, scoreFirst, scoreSecond, scoreThird;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload = follower.pathBuilder().addPath(new BezierLine(new Point(startPose),
                preloadDunkSpot))
                .setConstantHeadingInterpolation(0)
                .build();

        ferryFirst = follower.pathBuilder().addPath(new BezierCurve(preloadDunkSpot,
                new Point(0.797, 7.176, Point.CARTESIAN),
                new Point(111.628, 59.003, Point.CARTESIAN),
                new Point(77.502, 17.223, Point.CARTESIAN),
                firstFerrySpot))
                .setConstantHeadingInterpolation(0)
                .build();

        ferrySecond = follower.pathBuilder().addPath(new BezierCurve(firstFerrySpot,
                new Point(108.917, 25.674, Point.CARTESIAN),
                new Point(66.020, 10.684, Point.CARTESIAN),
                secondFerrySpot))
                .setConstantHeadingInterpolation(0)
                .build();

        grabFirst = follower.pathBuilder().addPath(new BezierLine(secondFerrySpot, firstGrabSpot))
                .setConstantHeadingInterpolation(0)
                .build();

        scoreFirst = follower.pathBuilder().addPath(new BezierCurve(firstGrabSpot,
                new Point(11.641, 65.063, Point.CARTESIAN),
                firstDunkSpot))
                .setConstantHeadingInterpolation(0)
                .build();

        grabSecond = follower.pathBuilder().addPath(new BezierLine(firstDunkSpot, generalGrabSpot))
                .setConstantHeadingInterpolation(0)
                .build();

        scoreSecond = follower.pathBuilder().addPath(new BezierLine(generalGrabSpot, secondDunkSpot))
                .setConstantHeadingInterpolation(0)
                .build();

        grabThird = follower.pathBuilder().addPath(new BezierLine(secondDunkSpot, generalGrabSpot))
                .setConstantHeadingInterpolation(0)
                .build();

        scoreThird = follower.pathBuilder().addPath(new BezierLine(generalGrabSpot, thirdDunkSpot))
                .setConstantHeadingInterpolation(0)
                .build();

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(ferryFirst,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(ferrySecond,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabFirst,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scoreFirst,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabSecond,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scoreSecond, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(grabThird,true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    follower.followPath(scoreThird);
                    setPathState(9);
                }

            case 9:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }


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
        telemetry.addData("isbusy?", follower.isBusy());
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
        follower.setStartingPose(startPose);
        buildPaths();
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

