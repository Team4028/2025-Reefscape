package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.util.Units;
import frc.robot.util.GetMotorData.MotorData;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double armEncoderRaw = 0.0;
        public double armEncoderRad = 0.0;
        public double armEncoderDeg = 0.0;
        public double armAngleRad = 0.0;
        public double armAngleDeg = 0.0;
        public double armVelocityRotPerSec = 0.0;
        public double armMotorVelocityRotPerSec = 0.0;
        public double appliedVoltage = 0.0;
        public double currentAmps = 0.0;
        public MotorData motorData = MotorData.empty();
    }

    public default void updateInputs(ArmIOInputs inputs) {
        inputs.armAngleDeg = Units.radiansToDegrees(inputs.armAngleRad);
        inputs.armEncoderDeg = Units.radiansToDegrees(inputs.armEncoderRad);
    }

    public default void setVoltage(double volts) {
    }

    public default void setVBus(double vBus) {
    }
}
