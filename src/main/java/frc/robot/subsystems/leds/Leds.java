package frc.robot.subsystems.leds;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {

    private final LedsIO io;
    private final LedsIOInputsAutoLogged inputs;
    private final LedsStateTracker stateTracker;
    private final Color[] targetColors = new Color[LedsConstants.NUM_LEDS];

    public static final record SimData(int[] ledColors) {}

    public enum Color {
        GREEN(0, 254, 0),
        PURPLE(118, 0, 254),
        PINK(254, 0, 118),
        YELLOW(118, 118, 0),
        ORANGE(254, 55, 0),
        LBLUE(55, 55, 254),
        BLUE(0, 0, 254),
        WHITE(254, 254, 254),
        RED(254, 0, 0),
        OFF(0, 0, 0);

        public final int r;
        public final int g;
        public final int b;
        public final int color;

        private Color(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
            color = Leds.rgbwToColor(r, g, b, 0);
        }
    }

    public Leds(LedsIO io) {
        this.io = io;
        inputs = new LedsIOInputsAutoLogged();
        stateTracker = new LedsStateTracker();
        Arrays.fill(targetColors, Color.OFF);
    }

    public static int rgbwToColor(int r, int g, int b, int w) {
        return r | (g << 8) | (b << 16) | (w << 24);
    }

    public static int[] colorToRGBW(int color) {
        return new int[] {
                color & 0xFF,
                (color >> 8) & 0xFF,
                (color >> 16) & 0xFF,
                (color >> 24) & 0xFF,
        };
    }

    public Command setToColor(Color color) {
        return runOnce(() -> {
            Arrays.fill(targetColors, color);
            stateTracker.state = LedsStates.SOLID_COLOR;
        });
    }

    public Command setToColors(Color... colors) {
        return runOnce(() -> {
            int len = colors.length >= targetColors.length ? 1 : Math.floorDiv(targetColors.length, colors.length);
            for (int i = 0; i < targetColors.length; i++)
                targetColors[i] = colors[Math.floorDiv(i, len)];
            stateTracker.state = LedsStates.SOLID_COLOR;
        });
    }

    @Override
    public void periodic() {
        stateTracker.state.execute(this);
        io.updateInputs(inputs);
        Logger.processInputs("Leds", inputs);
    }

    @CreateState("solid_color")
    public void fillSolidColor() {
        for (int i = 0; i < targetColors.length; i++)
            io.setLed(targetColors[i].r, targetColors[i].g, targetColors[i].b, 0, i);
    }

    public SimData getSimData() {
        return new SimData(inputs.ledColors);
    }
}
