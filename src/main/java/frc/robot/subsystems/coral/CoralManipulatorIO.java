package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.GetMotorData.MotorData;

public interface CoralManipulatorIO {
    @AutoLog
    public static class CoralManipulatorIOInputs {
        public double currentAmps = 0.0;
        public double appliedVolts = 0.0;
        public MotorData motorData = MotorData.empty();
    }

    public default void updateInputs(CoralManipulatorIOInputs inputs) {
    }

    public default void setVoltage(double volts) {
    }

    public default void setVbus(double vBus) {
    }
}
