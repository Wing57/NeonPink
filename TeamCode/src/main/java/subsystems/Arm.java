package subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private Servo pitch, twist, claw, left, right;

    public Arm(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        pitch = hardwareMap.get(Servo.class, "pitch");
        twist = hardwareMap.get(Servo.class, "twist");
        claw = hardwareMap.get(Servo.class, "claw");
    }

    public void setArm(double pos) {
        left.setPosition(pos);
        right.setPosition(pos);
    }

    public void setPitch(double pos) {
        pitch.setPosition(pos);
    }

    public void setTwist(double pos) {
        twist.setPosition(pos);
    }

    public void setClaw(double pos) {
        claw.setPosition(pos);
    }

    public void updateServos() {

    }

}
