package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        ThreeWheelIMUConstants.forwardTicksToInches = .002;
        ThreeWheelIMUConstants.strafeTicksToInches = .002;
        ThreeWheelIMUConstants.turnTicksToInches = .002;
        ThreeWheelIMUConstants.leftY = 4.8325;
        ThreeWheelIMUConstants.rightY = -4.8325;
        ThreeWheelIMUConstants.strafeX = 1.542;
        //4.8235
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "leftArm";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "br";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "leftExt";
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
    }
}




