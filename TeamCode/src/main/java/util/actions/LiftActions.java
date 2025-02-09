package util.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import pedroPathing.constants.Constants;
import subsystems.Lift;

public class LiftActions {

    public final Lift lift;
    public final LiftA liftA;
    public LiftActions(Lift lift) {
        this.lift = lift;
        this.liftA = new LiftA();

    }

    //TODO: why return t
    public class LiftA {
        public Action setPivotPosition(double pos) {
            return t -> {
                lift.setTargetV(pos);
                return false;
            };
        }

        public Action setExtendPosition(double pos) {
            return t -> {
                lift.setTargetX(pos);
                return false;
            };
        }

        public Action waitUntilFinishedV() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    return !lift.pivotAtSetpoint();
                }
            };
        }

        public Action waitUntilFinishedX() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    return !lift.extendAtSetpoint();
                }
            };
        }

        public Action pivotUp() {
            return setPivotPosition(Constants.Lift.UP);
        }

        public Action pivotSpecimen() { return setPivotPosition(300); }

        public Action pivotInit() { return setPivotPosition(1200); };

        public Action pivotDown() {
            return setPivotPosition(Constants.Lift.DOWN);
        }

        public Action extendZero() {
            return setExtendPosition(Constants.Lift.AUTO_ZERO);
        }

        public Action extendSpecimen() {
            return setExtendPosition(Constants.Lift.CLIP);
        }
    }
}
