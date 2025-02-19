package frc.robot.subsystems.leds;

import java.util.Arrays;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.limelight.Limelight;

public class Leds extends SubsystemBase {
    public final CANdle candle;
    private Color color;
    private boolean seeAnyTag = false;

    public enum Color {
        RED(254, 0, 0),
        ORANGE(254, 55, 0),
        YELLOW(118, 118, 0),
        GREEN(0, 254, 0),
        LBLUE(55, 55, 254),
        BLUE(0, 0, 254),
        PURPLE(118, 0, 254),
        PINK(254, 0, 118),
        WHITE(254, 254, 254),

        OFF(0, 0, 0);

        public int r;
        public int g;
        public int b;
        public int color;

        Color(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
            this.color = r | g << 8 | b << 16;
        }
    }

    public Leds() {
        candle = new CANdle(21, "rio");
        candle.configBrightnessScalar(.25);
        candle.configLEDType(LEDStripType.GRB);
        setColor(Color.WHITE);
    }

    public Command limelightToLeds(int aprilTagCount) {
        if (aprilTagCount == 1) {
            return seesAprilTag();
        } else if (aprilTagCount == 2) {
            return seeTwoAprilTag();
        } else if (aprilTagCount == 3) {
            return seeThreeAprilTag();
        } else if (aprilTagCount == 4) {
            return seeFourAprilTag();
        } else if (aprilTagCount > 4) {
            return seeMoreAprilTag();
        } else {
            return setNoColorCommand();
        }
    }

    public void setLedsColor(int ledIndex, int count, int color) {
        setLedsColor(ledIndex, count, color & 0xFF, (color >> 8) & 0xFF, (color >> 16) & 0xFF);
    }

    public void setLedsColor(int ledIndex, int count, int r, int g, int b) {
        candle.setLEDs(r, g, b, 0, ledIndex, count);
    }

    public void setColor(Color color) {
        this.color = color;
        candle.animate(null);
        candle.setLEDs(this.color.r, this.color.g, this.color.b);
    }

    public void setColor(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    public Command setNoColorCommand() {
        return runOnce(() -> setColor(Color.OFF));
    }

    public Command setColorRedCommand() {
        return runOnce(() -> setColor(Color.RED));
    }

    public Command setColorOrangeCommand() {
        return runOnce(() -> setColor(Color.ORANGE));
    }

    public Command setColorYellowCommand() {
        return runOnce(() -> setColor(Color.YELLOW));
    }

    public Command setColorGreenCommand() {
        return runOnce(() -> setColor(Color.GREEN));
    }

    public Command setColorLBlueCommand() {
        return runOnce(() -> setColor(Color.LBLUE));
    }

    public Command setColorBlueCommand() {
        return runOnce(() -> setColor(Color.BLUE));
    }

    public Command setColorPinkCommand() {
        return runOnce(() -> setColor(Color.PINK));
    }

    public Command setColorPurpleCommand() {
        return runOnce(() -> setColor(Color.PURPLE));
    }

    public Command setColorWhiteCommand() {
        return runOnce(() -> setColor(Color.WHITE));
    }

    public Command hasCoralAnimation() {
        return Commands.repeatingSequence(setColorGreenCommand(), setNoColorCommand(), setNoColorCommand(),
                setColorGreenCommand());
    }

    public Command gettingCoralAnimation() {
        return Commands.repeatingSequence(setColorRedCommand(), setNoColorCommand(), setNoColorCommand(),
                setColorRedCommand());
    }

    public Command gettingCoralAnimationSlow() {
        return Commands.repeatingSequence(setColorRedCommand().andThen(Commands.waitSeconds(.5))
                .andThen(setNoColorCommand()).andThen(Commands.waitSeconds(.5)));
    }

    public Command shootingCoralAnimation() {
        return Commands.repeatingSequence(setColorPurpleCommand(), setNoColorCommand(), setNoColorCommand(),
                setColorPurpleCommand());
    }

    public Command seesAprilTag() {
        return Commands.repeatingSequence(setColorWhiteCommand().andThen(Commands.waitSeconds(.25))
                .andThen(setNoColorCommand()).andThen(Commands.waitSeconds(.115)));
    }

    public Command seeTwoAprilTag() {
        return Commands.repeatingSequence(setColorGreenCommand().andThen(Commands.waitSeconds(.25))
                .andThen(setNoColorCommand()).andThen(Commands.waitSeconds(.115)));
    }

    public Command seeThreeAprilTag() {
        return Commands.repeatingSequence(setColorPurpleCommand().andThen(Commands.waitSeconds(.25))
                .andThen(setNoColorCommand()).andThen(Commands.waitSeconds(.115)));
    }

    public Command seeFourAprilTag() {
        return Commands.repeatingSequence(setColorRedCommand().andThen(Commands.waitSeconds(.25))
                .andThen(setNoColorCommand()).andThen(Commands.waitSeconds(.115)));
    }

    public Command seeMoreAprilTag() {
        return Commands.repeatingSequence(setColorBlueCommand().andThen(Commands.waitSeconds(.25))
                .andThen(setColorGreenCommand()).andThen(Commands.waitSeconds(.25)));
    }

    public Command sevenAnimation() {
        return Commands.repeatingSequence(setColorRedCommand()
                .andThen(Commands.waitSeconds(.65).andThen(setNoColorCommand()).andThen(Commands.waitSeconds(.05))
                        .andThen(setColorRedCommand()).andThen(Commands.waitSeconds(.25))
                        .andThen(setColorYellowCommand())
                        .andThen(Commands.waitSeconds(.35))
                        .andThen(setColorRedCommand()).andThen(Commands.waitSeconds(.35))
                        .andThen(setColorOrangeCommand()).andThen(Commands.waitSeconds(.25))
                        .andThen(setColorGreenCommand()).andThen(Commands.waitSeconds(.9))
                        .andThen(setColorBlueCommand()).andThen(Commands.waitSeconds(1)).andThen(setNoColorCommand())
                        .andThen(Commands.waitSeconds(.4))
                        .andThen(setColorRedCommand())
                        .andThen(Commands.waitSeconds(.65)).andThen(setNoColorCommand())
                        .andThen(Commands.waitSeconds(.05))
                        .andThen(setColorRedCommand()).andThen(Commands.waitSeconds(.25))
                        .andThen(setColorYellowCommand()).andThen(Commands.waitSeconds(.35))
                        .andThen(setColorRedCommand()).andThen(Commands.waitSeconds(.35))
                        .andThen(setColorOrangeCommand()).andThen(Commands.waitSeconds(.25))
                        .andThen(setColorGreenCommand()).andThen(Commands.waitSeconds(.4))
                        .andThen(setColorOrangeCommand()).andThen(Commands.waitSeconds(.5))
                        .andThen(setColorGreenCommand()).andThen(Commands.waitSeconds(.25))
                        .andThen(setColorBlueCommand()).andThen(Commands.waitSeconds(.6))
                        .andThen(setColorPurpleCommand()).andThen(Commands.waitSeconds(.8))));
    }

    public void updateLL(Limelight... ll) {
        seeAnyTag = Arrays.stream(ll).anyMatch(Limelight::getTV);
    }

    public int totalTags(Limelight... ll) {
        return Arrays.stream(ll).mapToInt(Limelight::getTargetCount).sum();
    }

    public Trigger sendLimeColors() {
        return new Trigger(() -> seeAnyTag);
    }
}
