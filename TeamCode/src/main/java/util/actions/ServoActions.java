package util.actions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.robotcore.external.Const;

import pedroPathing.constants.Constants;
import subsystems.Arm;

public class ServoActions {
    public final Arm arm;
    public final ArmA armA;

    public ServoActions(Arm arm) {
        this.arm = arm;
        this.armA = new ArmA();
    }

    public Action intakeSpecimen() {
        return new ParallelAction(
                armA.abnormal(),
                armA.pitchSpecimen(),
                armA.open(),
                armA.armSpecimen()
        );
    }

    public Action acquireSpecimen() {
        return new SequentialAction(
                armA.close(),
                new SleepAction(0.3),
                armA.armClip(),
                armA.normal()
        );
    }

    public Action scoreSpecimenDrag() {
        return new SequentialAction(
                armA.armGanch(),
                new SleepAction(1),
                armA.open()
        );
    }

    public Action scoreSpecimen() {
        return new SequentialAction(
                armA.armGanch(),
                new SleepAction(0.6),
                armA.open()
        );
    }


    public class ArmA {
        public Action setClaw(double pos) {
            return t -> {
                arm.setClaw(pos);
                return false;
            };
        }

        public Action setArm(double pos) {
            return t -> {
                arm.setArm(pos);
                return false;
            };
        }

        public Action setPitch(double pos) {
            return t -> {
                arm.setPitch(pos);
                return false;
            };
        }

        public Action setTwist(double pos) {
            return t -> {
                arm.setTwist(pos);
                return false;
            };
        }

        public Action setSweep(double pos) {
            return t -> {
                arm.sweep(pos);
                return false;
            };
        }

        public Action open() {
            return setClaw(Constants.Servos.OPEN);
        }

        public Action close() {
            return setClaw(Constants.Servos.CLOSE);
        }

        public Action pitchSpecimen() {
            return setPitch(Constants.Servos.PITCH_SPECIMEN);
        }

        public Action pitchGrab() {
            return setPitch(Constants.Servos.PITCH_GRAB);
        }

        public Action pitchBucket() {
            return setPitch(Constants.Servos.PITCH_BUCKET);
        }

        public Action armSpecimen() {
            return setArm(Constants.Servos.ARM_SPECIMEN);
        }

        public Action armClip() {
            return setArm(Constants.Servos.ARM_BUCKET);
        }

        public Action armInit() { return setArm(0.5); }

        public Action armSearch() { return setArm(Constants.Servos.ARM_SEARCH); }

        public Action armGrab() { return setArm(Constants.Servos.ARM_INTAKE); }

        public Action armBucket() { return setArm(Constants.Servos.ARM_BUCKET); }




        public Action pitchInit() { return setPitch(Constants.Servos.PITCH_AUTO_INIT); }

        public Action armGanch() {
            return setArm(Constants.Servos.ARM_CLIP);
        }

        public Action normal() {
            return setTwist(Constants.Servos.TWIST_NORMAL);
        }

        public Action abnormal() {
            return setTwist(Constants.Servos.TWIST_ABNORMAL);
        }

        public Action sweepDown() {
            return setSweep(Constants.Servos.SWEEP_DOWN);
        }

        public Action sweepUp() {
            return setSweep(Constants.Servos.SWEEP_UP);
        }

        public Action sweepInit() {
            return setSweep(0.35);
        }
    }
}
