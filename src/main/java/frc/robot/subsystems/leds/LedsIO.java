package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LedsIO {

    @AutoLog
    public static class LedsIOInputs {
        public int[] ledColors = new int[LedsConstants.NUM_LEDS];
    }

    public default void updateInputs(LedsIOInputs inputs) {}

    public default void setLeds(int r, int g, int b) {}
    public default void setLeds(int r, int g, int b, int w, int startIdx, int count) {}
    public default void setLed(int r, int g, int b, int w, int idx) {}
}
