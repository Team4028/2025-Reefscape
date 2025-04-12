package frc.robot.subsystems.groundinfeed;

import org.littletonrobotics.junction.AutoLog;

public interface GrondTOFIO {

    @AutoLog
    public static class GrondTOFIOInputs {
        public double range = 0;
        public double rangeSigma = 0;
        public double samplingTime = 0;
        public double lightingLevel = 0;
        public boolean rangeValid = false;
    }

    public default void updateInputs(GrondTOFIOInputs inputs) {
    }

    public default void setRangeOfI(int topX, int topY, int bottomX, int bottomY) {
    }
}
