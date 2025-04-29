package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.util.Units;
import frc.robot.util.MotorData;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public double armEncoderRaw = 0.0;
        public double armEncoderRad = 0.0;
        public double armEncoderDeg = 0.0;
        public double armMotorPositionRaw = 0.0;
        public double armAngleRad = 0.0;
        public double armAngleDeg = 0.0;
        public double armVelocityRotPerSec = 0.0;
        public double armMotorVelocityRotPerSec = 0.0;
        public double appliedVoltage = 0.0;
        public double currentAmps = 0.0;
        public double canMagPosition = 0.0;
        public double canMagVelocity = 0.0;
        public boolean canMagInRange = false;
        public MotorData motorData = MotorData.empty();
        public boolean isConnected = false;
    }

    default void updateInputs(ArmIOInputs inputs) {
        inputs.armAngleDeg = Units.radiansToDegrees(inputs.armAngleRad);
        inputs.armEncoderDeg = Units.radiansToDegrees(inputs.armEncoderRad);
    }

    default void setVoltage(double volts) {
    }

    default void setVBus(double vBus) {
    }

    default void setPID(double position) {
    }
}
