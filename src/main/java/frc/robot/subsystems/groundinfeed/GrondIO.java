package frc.robot.subsystems.groundinfeed;

import frc.robot.util.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface GrondIO {
    @AutoLog
    public class GrondIOInputs {
        public double appliedVoltage = 0.0;
        public double currentAmps = 0.0;
        public double positionRot = 0.0;
        public double velocityRotPerSec = 0.0;
        public MotorData motorData = MotorData.empty();
        public boolean isConnected = false;
    }

    public default void updateInputs(GrondIOInputs inputs) {
    }

    public default void setVbus(double vbus) {
    }

    public default void setPosition(double posMRot) {

    }

    public default void setVoltage(double voltage) {
    }

    public default void setCurrent(double amps) {
    }

    default void directSetMotor(double vbus) {
    }
}
