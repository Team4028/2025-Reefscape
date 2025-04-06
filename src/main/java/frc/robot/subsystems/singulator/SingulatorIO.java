package frc.robot.subsystems.singulator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.MotorData;

public interface SingulatorIO {

    @AutoLog
    public static class SingulatorIOInputs {
        public double currentAmps = 0.0;
        public double appliedVolts = 0.0;
        public MotorData motorData;
    }

    public default void updateInputs(SingulatorIOInputs inputs) {

    }

    public default void setVoltage(double voltage) {
    }

    public default void setVBus(double vbus) {
    }
}
