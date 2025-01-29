package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double armEncoderRaw = 0.0;
        public double armEncoderRad = 0.0;
        public double armAngleRad = 0.0;
        public double armVelocityRotPerSec = 0.0;
        public double armMotorVelocityRotPerSec = 0.0;
        public double appliedVoltage = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {
    }

    public default void setVoltage(double volts) {
    }

    public default void setVBus(double vBus) {
    }
}
