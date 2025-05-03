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

@Autonomous(name = "5 nugget", group = "Examples")
public class FiveNuggets extends ActionOpMode {

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
    private final Pose startPose = new Pose(10, 57, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    public PathChain scorePreload, scoreSecond, firstFerry, secondFerry, scoreFirst, grabSecond, grabThird, scoreThird, park;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        scorePreload = follower.pathBuilder().addPath(new BezierLine(new Point(10, 57, Point.CARTESIAN),
                        new Point(42, 70, Point.CARTESIAN)))
                .addTemporalCallback(0, ()-> run(new SequentialAction(liftActions.liftA.pivotSpecimen(), servoActions.armA.armClip(), servoActions.armA.pitchSpecimen(), new SleepAction(0.3), liftActions.liftA.extendSpecimen())))
                .setConstantHeadingInterpolation(0).build();

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        firstFerry = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(42.000, 70.000, Point.CARTESIAN),
                        new Point(18.923, 29.077, Point.CARTESIAN),
                        new Point(24.000, 42.000, Point.CARTESIAN),
                        new Point(56.538, 39.462, Point.CARTESIAN),
                        new Point(60.000, 26.077, Point.CARTESIAN)))
                .addTemporalCallback(0.3, ()-> run(liftActions.liftA.extendZero()))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(60.000, 26.077, Point.CARTESIAN),
                        new Point(23.308, 25.615, Point.CARTESIAN)))
                //  .addParametricCallback(.2, ()-> run (liftActions.liftA.pivotUp()))
                .setConstantHeadingInterpolation(0)                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        secondFerry = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(23.308, 25.615, Point.CARTESIAN),
                        new Point(58.154, 32.769, Point.CARTESIAN),
                        new Point(60.231, 14.538, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(60.231, 14.538, Point.CARTESIAN),
                        new Point(15.5, 14.538, Point.CARTESIAN)))
                .addParametricCallback(.2, ()-> run(new ParallelAction(liftActions.liftA.extendZero(), servoActions.intakeSpecimen())))

                .setConstantHeadingInterpolation(0)                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreFirst = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(15.5, 14.538, Point.CARTESIAN),
                        new Point(20.308, 65.769, Point.CARTESIAN),
                        new Point(43.2, 69.500, Point.CARTESIAN)))
                .addTemporalCallback(0, () -> run(new ParallelAction(liftActions.liftA.extendSpecimen())))

                .setConstantHeadingInterpolation(0)                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabSecond = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(42.4, 69.500, Point.CARTESIAN),
                        new Point(23.308, 73.615, Point.CARTESIAN),
                        new Point(45.692, 23.077, Point.CARTESIAN),
                        new Point(15.5, 29, Point.CARTESIAN)))
                .addParametricCallback(.2, ()-> run(new ParallelAction(liftActions.liftA.extendZero(), servoActions.intakeSpecimen(), liftActions.liftA.pivotUp())))
                .setConstantHeadingInterpolation(0)                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        scoreSecond = follower.pathBuilder().addPath(new BezierLine(new Point(15.5, 29, Point.CARTESIAN), /* Control Point */ new Point(43.4, 71.000)))
                .addParametricCallback(0.2, ()-> run(new ParallelAction(liftActions.liftA.extendSpecimen())))
                .setConstantHeadingInterpolation(0)
                .build();

        grabThird = follower.pathBuilder().addPath(new BezierCurve(new Point(42.400, 71.000, Point.CARTESIAN),
                        new Point(23.308, 73.615, Point.CARTESIAN),
                        new Point(45.692, 23.077, Point.CARTESIAN),
                        new Point(15.5, 29.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(0)
                .addParametricCallback(.2, ()-> run(new ParallelAction(liftActions.liftA.extendZero(), servoActions.intakeSpecimen(), liftActions.liftA.pivotUp())))
                .build();

        scoreThird = follower.pathBuilder().addPath(new BezierLine(new Point(15.5, 29, Point.CARTESIAN), /* Control Point */ new Point(43.4, 73.000)))
                .addParametricCallback(0.2, ()-> run(new ParallelAction(liftActions.liftA.extendSpecimen(), servoActions.armA.normal())))
                .setConstantHeadingInterpolation(0)
                .build();

        park = follower.pathBuilder().addPath(new BezierCurve(new Point(42.400, 73.000, Point.CARTESIAN),
                new Point(10.846, 74.077, Point.CARTESIAN),
                new Point(40.846, 21.462, Point.CARTESIAN),
                new Point(59.077, 21.692, Point.CARTESIAN),
                new Point(58.154, 9, Point.CARTESIAN)))
                .addParametricCallback(0.4, ()-> run(new ParallelAction(liftActions.liftA.extendZero(), servoActions.armA.armClip())))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine( new Point(58.154, 9, Point.CARTESIAN),
                        new Point(11.000, 9, Point.CARTESIAN)))
                .addParametricCallback(0.4, ()-> run(servoActions.armA.armGanch()))
                .setConstantHeadingInterpolation(0)
                .build();

    }

    private void buildTaskList() {
        tasks.clear();

        PathChainTask preloadTask = new PathChainTask(scorePreload, 1)
                .addWaitAction(0.6, servoActions.scoreSpecimenDrag());
        tasks.add(preloadTask);

        //TODO: tbh could merge ferries no?
        PathChainTask firstFerryTask = new PathChainTask(firstFerry, 0.2)
                .addWaitAction(0, liftActions.liftA.pivotUp());
        tasks.add(firstFerryTask);

        PathChainTask secondFerryTask = new PathChainTask(secondFerry, 0.8)
                .addWaitAction(0.5, servoActions.acquireSpecimen());
        tasks.add(secondFerryTask);

        PathChainTask scoreFirstTask = new PathChainTask(scoreFirst, 0.9)
                .addWaitAction(0.5, servoActions.scoreSpecimen());
        tasks.add(scoreFirstTask);

        PathChainTask grabSecondTask = new PathChainTask(grabSecond, 0.5)
                .addWaitAction(0.3, servoActions.acquireSpecimen());
        tasks.add(grabSecondTask);

        PathChainTask scoreSecondTask = new PathChainTask(scoreSecond, 0.9)
                .addWaitAction(0.65, servoActions.scoreSpecimen());
        tasks.add(scoreSecondTask);

        PathChainTask grabThirdTask = new PathChainTask(grabThird, .5)
                .addWaitAction(0.3, servoActions.acquireSpecimen());
        tasks.add(grabThirdTask);

        PathChainTask scoreThirdTask = new PathChainTask(scoreThird, 0.9)
                .addWaitAction(0.6, servoActions.scoreSpecimen());
        tasks.add(scoreThirdTask);

        PathChainTask pathTask = new PathChainTask(park, 0);
        tasks.add(pathTask);
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

        run(new ParallelAction(servoActions.armA.close(), servoActions.armA.armClip(), servoActions.armA.normal(), servoActions.armA.pitchSpecimen(), servoActions.armA.armInit(), liftActions.liftA.pivotInit(), servoActions.armA.pitchInit()));

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

