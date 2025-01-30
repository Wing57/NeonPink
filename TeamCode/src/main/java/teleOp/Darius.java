package teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import pedroPathing.constants.Constants.Servos;

import pedroPathing.constants.Constants;


import java.util.Arrays;
import java.util.List;

import util.navx.AHRS;

@TeleOp
public class Darius extends LinearOpMode {

    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx extend1, extend2, leftPivot, rightPivot;
    private List<DcMotorEx> allmotors;

    private Servo pitch, twist, claw, left, right, leftLED, rightLED;


    public static PIDController controllerX, controllerV;
    public static double dx = 0.00000;
    public static double ix = 0.000;
    public static double pv = 0.006;
    public static double px = 0.0006;
    public static double iv = 0.005;

    public static double dv = 0.00004;
    public static double f = 0.25;
    public static double targetX, targetV = 0;
    private double twistPos = Servos.TWIST_NORMAL;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    //private IMU imu;
    private AHRS navx;

    public volatile double yaw;

    private enum States {
        START,
        SEARCHING,
        SPECIMEN,
        CLIP,
        GANCH,
        GRABBING,
        STORE,
        FEED,
        LOW,
        HIGH,
        CLIMB
    }

    States state = States.SEARCHING;

    private ElapsedTime eventTimer;
    boolean update = false;
    boolean updateAlter = true;

    @Override
    public void runOpMode() throws InterruptedException {
        targetX = 5000;
        targetV = 0;
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        /*imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot)); */

        navx = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData);

        extend1 = hardwareMap.get(DcMotorEx.class, "leftExt");
        extend2 = hardwareMap.get(DcMotorEx.class, "rightExt");
        leftPivot = hardwareMap.get(DcMotorEx.class, "leftArm");
        rightPivot = hardwareMap.get(DcMotorEx.class, "rightArm");

        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        pitch = hardwareMap.get(Servo.class, "pitch");
        twist = hardwareMap.get(Servo.class, "twist");
        claw = hardwareMap.get(Servo.class, "claw");
        leftLED = hardwareMap.get(Servo.class, "leftled");
        rightLED = hardwareMap.get(Servo.class, "rightled");


        eventTimer = new ElapsedTime();


        allmotors = Arrays.asList(
                extend1, extend2,
                leftPivot, rightPivot,
                fl, fr, bl, br
        );

        allmotors.forEach(motor -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });

        leftPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        extend1.setDirection(DcMotorSimple.Direction.REVERSE);

        controllerX = new PIDController(px, ix, dx);
        controllerV = new PIDController(pv, iv, dv);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        new Thread(new gyroReader()).start();

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            drive(y, x, rx);

            if (gamepad1.start) {
                navx.zeroYaw();
            }

            liftControl();
            twist.setPosition(twistPos);

            leftLED.setPosition(0.722);
            rightLED.setPosition(0.722);


            stateHandler();

            telemetry.addData("extend: ", getExtension());
            telemetry.addData("pivot: ", getRawPivot());
            telemetry.addData("pivot degrees", toDegrees(getRawPivot()));
            telemetry.addData("pivot target", targetV);
            //telemetry.addData("heading", pinpoint.getHeading());
            telemetry.addData("state:", state.name());
            telemetry.update();
        }

    }

    public void stateHandler() {
        switch (state) {
            case SEARCHING:
                claw.setPosition(Constants.Servos.OPEN);
                targetV = Constants.Lift.DOWN;
                pitch.setPosition(Servos.PITCH_GRAB);
                setArm(Constants.Servos.ARM_SEARCH);

                if (gamepad1.right_trigger > 0.1) {
                    twistPos = 0.44;
                }
                if (gamepad1.left_trigger > 0.1 ){
                    twistPos = Servos.TWIST_NORMAL;
                }

                extensionControl(gamepad1.left_bumper, gamepad1.right_bumper);
                // twistControl(gamepad1.left_trigger > 0.1, gamepad1.right_trigger > 0.1);

                if (gamepad1.left_stick_button) {
                    targetX = Constants.Lift.SUBMERSIBLE;
                }

                if (gamepad1.x) {
                    setArm(Constants.Servos.ARM_INTAKE);

                    eventTimer.reset();
                    update = true;
                    state = States.GRABBING;
                }

                if (gamepad1.y) {
                    state = States.SPECIMEN;
                }
                break;

            case SPECIMEN:
                twistPos = Servos.TWIST_ABNORMAL;
                targetV = Constants.Lift.UP;
                targetX = Constants.Lift.SPECIMEN;

                if (gamepad1.right_bumper) {
                    claw.setPosition(Constants.Servos.OPEN);
                } else if (!gamepad1.b) {
                    claw.setPosition(Constants.Servos.CLOSE);
                }


                setArm(Constants.Servos.ARM_SPECIMEN);
                pitch.setPosition(Constants.Servos.PITCH_SPECIMEN);

                if (gamepad1.a) {
                    setArm(0.5);
                    targetX = 1000;
                    eventTimer.reset();
                    update = false;
                    updateAlter = false;
                    twistPos = Servos.TWIST_NORMAL;
                    state = States.CLIP;
                }

                if (gamepad1.back) {
                    state = States.SEARCHING;
                }

                break;

            case CLIP:

                if (gamepad1.right_stick_button && !update) {
                    targetX = Constants.Lift.CLIP;
                    update = true;
                    eventTimer.reset();

                }

                if (gamepad1.right_stick_button && update && eventTimer.time() > 0.25) {
                    setArm(Constants.Servos.ARM_CLIP);
                    state = States.GANCH;
                }

                break;

            case GANCH:
                if (gamepad1.back) {
                    setArm(0.5);
                    state = States.CLIP;
                }
                if (gamepad1.left_stick_button) {
                    claw.setPosition(Constants.Servos.OPEN);
                    targetX = Constants.Lift.AXEL_ZERO;
                    eventTimer.reset();
                    update = true;
                }

                if (inBounds(Constants.Lift.AXEL_ZERO, getExtension(), 2000) && update && eventTimer.time() > 1.0) {
                    state = States.SEARCHING;
                }

                if (gamepad1.y) {
                    claw.setPosition(Constants.Servos.OPEN);
                    updateAlter = true;
                    eventTimer.reset();
                }

                if (updateAlter && eventTimer.time() > 0.5) {
                    state = States.SPECIMEN;
                }

                break;
            case GRABBING:
                if (eventTimer.time() > 0.25 && update) {
                    claw.setPosition(Constants.Servos.CLOSE);
                    update = false;
                }

                if (gamepad1.b) {
                    claw.setPosition(Constants.Servos.OPEN);
                    state = States.SEARCHING;
                }

                if (eventTimer.time() > 0.3 && gamepad1.x) {
                    pitch.setPosition(Constants.Servos.PITCH_BUCKET);
                    state = States.STORE;
                }

                break;

            case STORE:
                targetX = Constants.Lift.ZERO;
                setArm(0.5);
                // pitch.setPosition(0.4);

                if (gamepad1.back) {
                    state = States.SEARCHING;
                }

                if (gamepad1.y) {
                    update = true;
                    eventTimer.reset();
                    state = States.HIGH;
                }

                if (gamepad1.a) {
                    state = States.FEED;
                }
                break;

            case FEED:
                targetV = Constants.Lift.UP;
                setArm(Servos.ARM_SPECIMEN);

                if (gamepad1.right_stick_button) {
                    claw.setPosition(Servos.OPEN);
                    twistPos = Servos.TWIST_NORMAL;
                    state = States.SEARCHING;
                }

                break;

            case HIGH:
                targetV = 0;
                twistPos = Servos.TWIST_ABNORMAL;

                if (inBounds(0, getRawPivot(), 400) && update) {
                    targetX = Constants.Lift.TOP_BUCKET;
                    setArm(Servos.ARM_BUCKET);
                    pitch.setPosition(Constants.Servos.PITCH_BUCKET);
                    update = false;
                }

                if (gamepad1.b) {
                    claw.setPosition(Constants.Servos.OPEN);
                }

                if (gamepad1.x) {
                    targetX = Constants.Lift.AXEL_ZERO;
                }

                if (inBounds(Constants.Lift.AXEL_ZERO, getExtension(), 2000) && !update && eventTimer.time() > 2) {
                    twistPos = Servos.TWIST_NORMAL;
                    state = States.SEARCHING;
                }

                break;


        }
    }

    public void liftControl() {
        controllerV.setPID(pv, iv, dv);
        //double pivotPos = toDegrees(extend2.getCurrentPosition());
        double pivotPos = getRawPivot();
        double pidV = controllerV.calculate(pivotPos, targetV);

        controllerX.setPID(px, ix, dx);
        double extensionPos = getExtension();
        double pidX = controllerX.calculate(extensionPos, targetX);

        leftPivot.setPower(pidV);
        rightPivot.setPower(pidV);
        extend1.setPower(pidX);
        extend2.setPower(pidX);
    }

    public void drive(double y, double x, double rx) {
        double botHeading = yaw;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);
    }

    public void setPivot(double pos) {
        left.setPosition(pos);
        right.setPosition(pos);
    }

    public void extensionControl(boolean low, boolean high) {
        if (high && getExtension() < Constants.Lift.TOP_BUCKET) {
            targetX += 1000;
        }

        if (low && getExtension() > Constants.Lift.ZERO) {
            targetX -= 1000;
        }
    }

    public void setArm(double pos) {
        left.setPosition(pos);
        right.setPosition(pos);
    }

    public void twistControl(boolean left, boolean right) {
        if (left && twistPos >= 0 && twistPos <= 1 ) {
            twistPos -= 0.05;
        }

        if (right && twistPos >= 0 && twistPos <= 1 ) {
            twistPos += 0.05;
        }
    }

    private double toDegrees(double pos) {
        return (360 / (8192 * 4.2)) * pos;
    }

    private double getExtension() {
        return rightPivot.getCurrentPosition();
    }

    private double getRawPivot() {
        return extend2.getCurrentPosition();
    }

    private boolean inBounds(double target, double currPos, double tolerance) {
        double error = Math.abs(currPos - target);
        return (error < tolerance);
    }

    private class gyroReader implements Runnable {
        @Override
        public void run() {
            while (!isStopRequested()) {
                double currentYaw;

                synchronized (Darius.this) {
                    currentYaw = -Math.toRadians(navx.getYaw());
                    yaw = currentYaw;
                }

                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }
}