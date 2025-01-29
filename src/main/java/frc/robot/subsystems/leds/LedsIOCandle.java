package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;

public class LedsIOCandle implements LedsIO {
    private final CANdle candle;
    private final int[] internalLedStates;

    public LedsIOCandle() {
        candle = new CANdle(LedsConstants.Candle.CAN_ID, LedsConstants.Candle.CANBUS);
        candle.configBrightnessScalar(LedsConstants.Candle.BRIGHTNESS_FACTOR);
        internalLedStates = new int[LedsConstants.NUM_LEDS];
    }

    @Override
    public void updateInputs(LedsIOInputs inputs) {
        inputs.ledColors = internalLedStates;
    }

    @Override
    public void setLeds(int r, int g, int b) {
        setLeds(r, g, b, 0, 0, LedsConstants.NUM_LEDS);
    }

    @Override
    public void setLeds(int r, int g, int b, int w, int startIdx, int count) {
        for (var i = startIdx; i < startIdx + count; i++) {
            internalLedStates[i] = Leds.rgbwToColor(r, g, b, w);
        }
        candle.setLEDs(r, g, b, w, startIdx, count);
    }

    @Override
    public void setLed(int r, int g, int b, int w, int idx) {
        setLeds(r, g, b, w, idx, 1);
    }
}
