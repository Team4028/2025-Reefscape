package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.GetMotorData.MotorData;

public interface AlgaeManipulatorIO {
    @AutoLog
    public static class AlgaeManipulatorIOInputs {
        public double currentAmps = 0;
        public double appliedVolts = 0;
        public double velocityRadPerSec = 0;
        public double accelRadPerSecPerSec = 0;
        public MotorData motorData = MotorData.empty();
    }

    public default void updateInputs(AlgaeManipulatorIOInputs inputs) {
    }

    public default void setVoltage(double volts) {
    }

    public default void setVbus(double vbus) {
    }
}
