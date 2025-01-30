package subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

public class Lift {

    private Telemetry telemetry;


    private DcMotorEx extend1, extend2, leftPivot, rightPivot;
    private List<DcMotorEx> allmotors;


    public static PIDController controllerX, controllerV;
    public static double dx = 0.00000;
    public static double ix = 0.000;
    public static double pv = 0.006;
    public static double px = 0.0006;
    public static double iv = 0.005;

    public static double dv = 0.00004;
    public static double f = 0.25;
    public static double targetX = 0, targetV = 0;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        extend1 = hardwareMap.get(DcMotorEx.class, "leftExt");
        extend2 = hardwareMap.get(DcMotorEx.class, "rightExt");
        leftPivot = hardwareMap.get(DcMotorEx.class, "leftArm");
        rightPivot = hardwareMap.get(DcMotorEx.class, "rightArm");

        leftPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        extend1.setDirection(DcMotorSimple.Direction.REVERSE);

        allmotors = Arrays.asList(
                extend1, extend2,
                leftPivot, rightPivot
        );

        allmotors.forEach(motor -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });

        controllerX = new PIDController(px, ix, dx);
        controllerV = new PIDController(pv, iv, dv);
    }

    //TODO: test tolerances
    public void updatePIDF() {
        controllerV.setPID(pv, iv, dv);
        controllerV.setTolerance(20);
        //double pivotPos = toDegrees(extend2.getCurrentPosition());
        double pivotPos = getRawPivot();
        double pidV = controllerV.calculate(pivotPos, targetV);

        controllerX.setPID(px, ix, dx);
        controllerX.setTolerance(400);
        double extensionPos = getExtension();
        double pidX = controllerX.calculate(extensionPos, targetX);

        /*
         double ticks_in_degrees = 537.7 / 360.0;
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
            double power = pid + ff;
         */

        leftPivot.setPower(pidV);
        rightPivot.setPower(pidV);
        extend1.setPower(pidX);
        extend2.setPower(pidX);
    }

    public void setTargetV(double pos) {
        targetV = pos;
    }

    public void setTargetX(double pos) {
        targetX = pos;
    }

    public double getExtension() {
        return rightPivot.getCurrentPosition();
    }

    public double getRawPivot() {
        return extend2.getCurrentPosition();
    }

    public void telemetry() {
        telemetry.addData("extend: ", getExtension());
        telemetry.addData("pivot: ", getRawPivot());
        telemetry.addData("extend target:", targetX);
        telemetry.addData("pivot target", targetV);
    }

}
