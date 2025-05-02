package auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.Arm;
import subsystems.Lift;
import util.actions.ActionOpMode;
import util.actions.LiftActions;
import util.actions.ServoActions;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Three Bucket", group = "Examples")
public class ThreeSample extends ActionOpMode {

    private static class WaitAction {
        double triggerTime; // seconds into the waiting phase
        Action action;
        boolean triggered;

        WaitAction(double triggerTime, Action action) {
            this.triggerTime = triggerTime;
            this.action = action;
            this.triggered = false;
        }
    }

    // -------------------------------------------------------------------------
    // PathChainTask: holds a PathChain (with param callbacks) and a WAIT phase
    // -------------------------------------------------------------------------
    private static class PathChainTask {
        PathChain pathChain;
        double waitTime; // how long to wait after the chain
        List<WaitAction> waitActions = new ArrayList<>();

        PathChainTask(PathChain pathChain, double waitTime) {
            this.pathChain = pathChain;
            this.waitTime = waitTime;
        }

        // Add a "wait action," triggered at a certain second in the WAIT phase
        PathChainTask addWaitAction(double triggerTime, Action action) {
            waitActions.add(new WaitAction(triggerTime, action));
            return this;
        }

        void resetWaitActions() {
            for (WaitAction wa : waitActions) {
                wa.triggered = false;
            }
        }
    }


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private Lift lift;
    private Arm arm;

    private LiftActions liftActions;
    private ServoActions servoActions;

    private static final double PATH_COMPLETION_T = 0.982;


    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private final List<PathChainTask> tasks = new ArrayList<>();
    private int currentTaskIndex = 0;


    private int taskPhase = 0;


    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8.307692307692308, 104.76923076923077, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    public PathChain scorePreload, grabFirst, scoreFirst, grabSecond, scoreSecond, park;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        scorePreload = follower.pathBuilder() .addPath(
                        new BezierLine(
                                new Point(8.308, 104.769, Point.CARTESIAN),
                                new Point(17.000, 125.385, Point.CARTESIAN)
                        )
                )
                .addTemporalCallback(0.1, () -> liftActions.liftA.pivotUp())
                .addParametricCallback(0.35, () -> liftActions.liftA.extendBucket())
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .addPath(
                        new BezierLine(
                                new Point(17, 125.385, Point.CARTESIAN),
                                new Point(16.000, 126.385, Point.CARTESIAN)
                        )
                )
                .build();

        grabFirst = follower.pathBuilder()    .addPath(
                        new BezierLine(
                                new Point(16.000, 126.385, Point.CARTESIAN),
                                new Point(30.923, 120.923, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .addParametricCallback(0.3, () -> liftActions.liftA.extendZero())
                .addParametricCallback(0.7, () -> new ParallelAction(liftActions.liftA.pivotDown(), servoActions.armA.normal(), servoActions.armA.pitchGrab(), servoActions.armA.armSearch()))
                .addParametricCallback(.9, () -> liftActions.liftA.extendAuto())
                .build();

        scoreFirst = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(8.308, 104.769, Point.CARTESIAN),
                                new Point(17.000, 125.385, Point.CARTESIAN)
                        )
                )
                .addTemporalCallback(0.1, () -> liftActions.liftA.pivotUp())
                .addParametricCallback(0.35, () -> liftActions.liftA.extendBucket())
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .addPath(
                        new BezierLine(
                                new Point(17, 125.385, Point.CARTESIAN),
                                new Point(16.000, 126.385, Point.CARTESIAN)
                        )
                )
                .build();

        grabSecond = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(17.000, 125.380, Point.CARTESIAN),
                                new Point(30.923, 132.231, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        scoreSecond = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(30.923, 132.231, Point.CARTESIAN),
                                new Point(17.000, 125.380, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(17, 125.38, Point.CARTESIAN),
                                new Point(20.000, 122.380, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(-45)                .build();




    }

    private void buildTaskList() {
        tasks.clear();

        PathChainTask preloadTask = new PathChainTask(scorePreload, 1)
                .addWaitAction(0.2, servoActions.armA.open());
        tasks.add(preloadTask);

        PathChainTask grabFirstTask = new PathChainTask(grabFirst, 1)
                .addWaitAction(0.3, servoActions.armA.armGrab())
                .addWaitAction(0.7, servoActions.armA.close());
        tasks.add(grabFirstTask);

        PathChainTask scoreFirstTask = new PathChainTask(scoreFirst, 1)
                .addWaitAction(0.2, servoActions.armA.open());
        tasks.add(scoreFirstTask);

        PathChainTask parkTask = new PathChainTask(park, 10)
                .addWaitAction(1, new ParallelAction(liftActions.liftA.extendZero()))
                .addWaitAction(3, liftActions.liftA.pivotUp());
        tasks.add(parkTask);


    }

    private void runTasks() {
        if (currentTaskIndex >= tasks.size()) {
            return; // all done
        }

        PathChainTask currentTask = tasks.get(currentTaskIndex);

        switch (taskPhase) {
            case 0: // == DRIVING ==
                // If we aren't following yet, start
                if (!follower.isBusy()) {
                    follower.followPath(currentTask.pathChain, true);
                    pathTimer.resetTimer();

                    // We only "reset" the *wait* actions here.
                    // Param-based callbacks are attached in the chain already.
                    currentTask.resetWaitActions();
                }

                double tValue = follower.getCurrentTValue(); // param progress [0..1]
                // NOTE: Param-based callbacks happen automatically in the `Follower`
                // when tValue crosses the callback thresholds.

                // Consider chain done at 99%
                if (tValue >= PATH_COMPLETION_T) {
                    // Move to WAIT
                    pathTimer.resetTimer();
                    taskPhase = 1;
                }
                break;

            case 1: // == WAITING ==
                double waitElapsed = pathTimer.getElapsedTimeSeconds();

                // Trigger any "wait actions" whose time has arrived
                for (WaitAction wa : currentTask.waitActions) {
                    if (!wa.triggered && waitElapsed >= wa.triggerTime) {
                        run(wa.action); // schedule this action
                        wa.triggered = true;
                    }
                }

                // Once we've fully waited out the entire waitTime, move on
                if (waitElapsed >= currentTask.waitTime) {
                    currentTaskIndex++;
                    taskPhase = 0;
                }
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        lift = new Lift(hardwareMap, telemetry);
        arm = new Arm(hardwareMap);

        liftActions = new LiftActions(lift);
        servoActions = new ServoActions(arm);

        run(new ParallelAction(servoActions.armA.close(), servoActions.armA.armClip(), servoActions.armA.abnormal(), servoActions.armA.pitchBucket(), servoActions.armA.armInit(), liftActions.liftA.pivotInit(), servoActions.armA.pitchInit()));

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        buildTaskList();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        currentTaskIndex = 0;
        taskPhase = 0;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        super.loop();

        // These loop the movements of the robot
        follower.update();

        runTasks();

        lift.updatePIDF();

        // Feedback to Driver Hub
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("isbusy?", follower.isBusy());
        telemetry.addData("progression:", follower.getCurrentTValue());
        telemetry.addData("Task Index", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("Wait Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Running Actions", runningActions.size());

        telemetry.update();
    }

    @Override
    public void init_loop() {
        super.loop();

        runTasks();

        lift.updatePIDF();
    }
}

