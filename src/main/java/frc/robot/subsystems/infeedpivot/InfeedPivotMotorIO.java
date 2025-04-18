package frc.robot.subsystems.infeedpivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.util.Units;

public interface InfeedPivotMotorIO {

    @AutoLog
    public static class InfeedPivotIOMotorInputs {
        public double positionRad = 0;
        public double positionDeg = 0;
        public double velRad = 0;
        public double velDeg = 0;
        public double appliedV = 0;
        public double currentA = 0;
        public boolean isConnected = false;
    }

    public default void updateInputs(InfeedPivotIOMotorInputs inputs) {
        inputs.positionDeg = Units.radiansToDegrees(inputs.positionRad);
        inputs.velDeg = Units.radiansToDegrees(inputs.velRad);
    }

    public default void zeroPosition(double zeroPosRad) {
    }

    public default void setVBus(double vbus) {
    }

    public default void setPid(double posRad) {
    }
}
