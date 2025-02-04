package frc.robot.subsystems.leds;

public class LedsIOSim implements LedsIO {
    private final int[] internalLedStates = new int[LedsConstants.NUM_LEDS];

    @Override
    public void updateInputs(LedsIOInputs inputs) {
        inputs.ledColors = internalLedStates;
    }

    @Override
    public void setLeds(int r, int g, int b, int w, int startIdx, int count) {
        for (int i = startIdx; i < startIdx + count; i++)
            internalLedStates[i] = Leds.rgbwToColor(r, g, b, w);
    }

    @Override
    public void setLeds(int r, int g, int b) {
        setLeds(r, g, b, 0, 0, LedsConstants.NUM_LEDS);
    }

    @Override
    public void setLed(int r, int g, int b, int w, int idx) {
        setLeds(r, g, b, 0, idx, 1);
    }
}
