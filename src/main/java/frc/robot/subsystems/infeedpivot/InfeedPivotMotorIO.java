package frc.robot.subsystems.infeedpivot;

import edu.wpi.first.math.util.Units;
import frc.robot.util.PIDStruct;
import org.littletonrobotics.junction.AutoLog;

public interface InfeedPivotMotorIO {

    @AutoLog
    class InfeedPivotIOMotorInputs {
        public double positionRad = 0;
        public double positionDeg = 0;
        public double velRad = 0;
        public double velDeg = 0;
        public double appliedV = 0;
        public double currentA = 0;
        public boolean isConnected = false;
    }

    default void updateInputs(InfeedPivotIOMotorInputs inputs) {
        inputs.positionDeg = Units.radiansToDegrees(inputs.positionRad);
        inputs.velDeg = Units.radiansToDegrees(inputs.velRad);
    }

    default void zeroPosition(double zeroPosRad) {
    }

    default void setVBus(double vbus) {
    }

    default void setPid(double posRad) {
    }

    default void resetPid(double posRad) {
    }

    default void setVoltage(double voltage) {
    }

    default void setPIDConstants(PIDStruct pid) {
    }
}
