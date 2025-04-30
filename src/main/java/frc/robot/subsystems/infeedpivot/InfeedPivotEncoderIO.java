package frc.robot.subsystems.infeedpivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.util.Units;

public interface InfeedPivotEncoderIO {

    @AutoLog
    class InfeedPivotEncoderIOInputs {
        public double positionRad = 0;
        public double positionDeg = 0;
        public double velocityRad = 0;
        public double velocityDeg = 0;
        public boolean connected = false;
    }

    default void updateInputs(InfeedPivotEncoderIOInputs inputs) {
        inputs.positionDeg = Units.radiansToDegrees(inputs.positionRad);
        inputs.velocityDeg = Units.radiansToDegrees(inputs.velocityRad);
    }
}