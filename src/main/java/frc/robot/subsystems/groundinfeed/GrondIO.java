package frc.robot.subsystems.groundinfeed;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.MotorData;

public interface GrondIO {
    @AutoLog
    public class GrondIOInputs {
        public double appliedVoltage = 0.0;
        public double currentAmps = 0.0;
        public MotorData motorData = MotorData.empty();
    }

    public default void updateInputs(GrondIOInputs inputs) {
    }

    public default void setVbus(double vbus) {
    }

    public default void setVoltage(double voltage) {
    }
}
