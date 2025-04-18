package frc.robot.subsystems.stick;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.MotorData;

public interface WhipStickIO {
    @AutoLog
    public static class WhipStickIOInputs {
        public double currentAmps = 0.0;
        public double appliedVolts = 0.0;
        public MotorData motorData = MotorData.empty();
        public boolean isConnected = false;
    }

    public default void updateInputs(WhipStickIOInputs inputs) {
    }

    public default void setVoltage(double volts) {
    }

    public default void setVbus(double vBus) {
    }
}
