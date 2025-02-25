package frc.robot.subsystems.leds;

import java.util.Arrays;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.limelight.Limelight;

public class Leds extends SubsystemBase {
    public final CANdle candle;
    private Color color;
    private CandleState candleState = CandleState.OFF;
    private StripState stripState = StripState.OFF;
    private boolean seeAnyTag = false;
    private Color canColor = Color.WHITE;
    private Color stripColor = Color.WHITE;
    private Timer canflasher;
    private Timer stripflasher;
    private boolean isCanOn = true;
    private boolean isStripOn = true;

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

    public enum CandleState {
        FLASH,
        OFF;
    }

    public enum StripState {
        FLASH,
        OFF;
    }

    public Leds() {
        candle = new CANdle(21, "rio");
        candle.configBrightnessScalar(.25);
        candle.configLEDType(LEDStripType.GRB);
        setCandleColor(Color.WHITE);
        setStripColor(Color.WHITE);
        canflasher = new Timer();
        stripflasher = new Timer();
    }

    public Command limelightToLeds(int aprilTagCount) {
        if (aprilTagCount == 1) {
            return seesAprilTagCandle();
        } else if (aprilTagCount == 2) {
            return seeTwoAprilTagCandle();
        } else if (aprilTagCount == 3) {
            return seeThreeAprilTagCandle();
        } else if (aprilTagCount == 4) {
            return seeFourAprilTagCandle();
        } else if (aprilTagCount > 4) {
            return seeMoreAprilTagCandle();
        } else {
            return setNoColorCandle();
        }
    }

    public void setLedsColor(int ledIndex, int count, int color) {
        setLedsColor(ledIndex, count, color & 0xFF, (color >> 8) & 0xFF, (color >> 16) & 0xFF);
    }

    public void setLedsColor(int ledIndex, int count, int r, int g, int b) {
        candle.setLEDs(r, g, b, 0, ledIndex, count);
    }

    public void setCandleColor(Color color) {
        this.color = color;
        candle.animate(null);
        candle.setLEDs(this.color.r, this.color.g, this.color.b, 0, 0, 8);
    }

    public void setStripColor(Color color) {
        this.color = color;
        candle.animate(null);
        candle.setLEDs(this.color.r, this.color.g, this.color.b, 0, 8, 60);
    }

    public void setallColors(Color color) {
        this.color = color;
        candle.animate(null);
        candle.setLEDs(this.color.r, this.color.g, this.color.b, 0, 0, 512);
    }

    public void setColorParts(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    public Command setNoColorCandle() {
        return runOnce(() -> setCandleColor(Color.OFF));
    }

    public Command setCandle(Color color) {
        return runOnce(() -> setCandleColor(color));
    }

    public Command setStrip(Color color) {
        return runOnce(() -> setCandleColor(color));
    }

    public Command flashCandle(Color color) {
        return Commands.repeatingSequence(setCandle(color), setCandle(Color.OFF));
    }

    public Command flashStrip(Color color) {
        return Commands.repeatingSequence(setCandle(color), setCandle(Color.OFF), setCandle(Color.OFF),
                setCandle(color));
    }

    public Command setNoColorStrip() {
        return runOnce(() -> setStripColor(Color.OFF));
    }

    public Command setNoColor() {
        return runOnce(() -> setallColors(Color.OFF));
    }

    public Command setCandleRedCommand() {
        return runOnce(() -> setCandleColor(Color.RED));
    }

    public Command setStripRedCommand() {
        return runOnce(() -> setStripColor(Color.RED));
    }

    public Command setCandleOrangeCommand() {
        return runOnce(() -> setCandleColor(Color.ORANGE));
    }

    public Command setStripOrangeCommand() {
        return runOnce(() -> setStripColor(Color.ORANGE));
    }

    public Command setCandleYellowCommand() {
        return runOnce(() -> setCandleColor(Color.YELLOW));
    }

    public Command setStripYellowCommand() {
        return runOnce(() -> setStripColor(Color.YELLOW));
    }

    public Command setCandleGreenCommand() {
        return runOnce(() -> setCandleColor(Color.GREEN));
    }

    public Command setStripGreenCommand() {
        return runOnce(() -> setStripColor(Color.GREEN));
    }

    public Command setCandleLBlueCommand() {
        return runOnce(() -> setCandleColor(Color.LBLUE));
    }

    public Command setStripLBlueCommand() {
        return runOnce(() -> setStripColor(Color.LBLUE));
    }

    public Command setCandleBlueCommand() {
        return runOnce(() -> setCandleColor(Color.BLUE));
    }

    public Command setStripBlueCommand() {
        return runOnce(() -> setStripColor(Color.BLUE));
    }

    public Command setCandlePinkCommand() {
        return runOnce(() -> setCandleColor(Color.PINK));
    }

    public Command setStripPinkCommand() {
        return runOnce(() -> setStripColor(Color.PINK));
    }

    public Command setCandlePurpleCommand() {
        return runOnce(() -> setCandleColor(Color.PURPLE));
    }

    public Command setStripPurpleCommand() {
        return runOnce(() -> setStripColor(Color.PURPLE));
    }

    public Command setCandleWhiteCommand() {
        return runOnce(() -> setCandleColor(Color.WHITE));
    }

    public Command setStripWhiteCommand() {
        return runOnce(() -> setStripColor(Color.WHITE));
    }

    public Command hasCoralAnimationCandle() {
        return Commands.sequence(setCandleBlueCommand(), setNoColorCandle(), setNoColorCandle(),
                setCandleBlueCommand());
    }

    public Command hasCoralAnimationStrip() {
        return Commands.sequence(setStripBlueCommand(), setNoColorStrip(), setNoColorStrip(),
                setStripBlueCommand());
    }

    public Command flashRedCandle() {
        return Commands.sequence(setCandleRedCommand(), setNoColorCandle(), setNoColorCandle(),
                setCandleRedCommand());
    }

    public Command flashRedStrip() {
        return Commands.sequence(setStripRedCommand(), setNoColorStrip(), setNoColorStrip(),
                setStripRedCommand());
    }

    public Command shootingCoralAnimationCandle() {
        return Commands.sequence(setCandlePurpleCommand(), setNoColorCandle(), setNoColorCandle(),
                setCandlePurpleCommand());
    }

    public Command shootingCoralAnimationStrip() {
        return Commands.sequence(setStripPurpleCommand(), setNoColorStrip(), setNoColorStrip(),
                setStripPurpleCommand());
    }

    public Command hasAlgaeAnimationCandle() {
        return Commands.sequence(setCandleGreenCommand(), setCandleWhiteCommand(), setCandleWhiteCommand(),
                setCandleGreenCommand());
    }

    public Command hasAlgaeAnimationStrip() {
        return Commands.sequence(setCandleGreenCommand(), setCandleWhiteCommand(), setCandleWhiteCommand(),
                setCandleGreenCommand());
    }

    public Command seesAprilTagCandle() {
        return Commands.sequence(setCandleWhiteCommand(), setNoColorCandle(), setNoColorCandle(),
                setCandleWhiteCommand());
    }

    public Command seeTwoAprilTagCandle() {
        return Commands.sequence(setCandleGreenCommand(), setNoColorCandle(), setNoColorCandle(),
                setCandleGreenCommand());
    }

    public Command seeThreeAprilTagCandle() {
        return Commands.sequence(setCandlePurpleCommand(), setNoColorCandle(), setNoColorCandle(),
                setCandlePurpleCommand());
    }

    public Command seeFourAprilTagCandle() {
        return Commands.sequence(setCandleRedCommand(), setNoColorCandle(), setNoColorCandle(), setCandleRedCommand());
    }

    public Command seeMoreAprilTagCandle() {
        return Commands.sequence(setCandleBlueCommand(), setCandleGreenCommand(), setCandleGreenCommand(),
                setCandleBlueCommand());
    }

    public Command sevenAnimationCandle() {
        return Commands.repeatingSequence(setCandleRedCommand()
                .andThen(Commands.waitSeconds(.65).andThen(setNoColorCandle()).andThen(Commands.waitSeconds(.05))
                        .andThen(setCandleRedCommand()).andThen(Commands.waitSeconds(.25))
                        .andThen(setCandleYellowCommand())
                        .andThen(Commands.waitSeconds(.35))
                        .andThen(setCandleRedCommand()).andThen(Commands.waitSeconds(.35))
                        .andThen(setCandleOrangeCommand()).andThen(Commands.waitSeconds(.25))
                        .andThen(setCandleGreenCommand()).andThen(Commands.waitSeconds(.9))
                        .andThen(setCandleBlueCommand()).andThen(Commands.waitSeconds(1)).andThen(setNoColorCandle())
                        .andThen(Commands.waitSeconds(.4))
                        .andThen(setCandleRedCommand())
                        .andThen(Commands.waitSeconds(.65)).andThen(setNoColorCandle())
                        .andThen(Commands.waitSeconds(.05))
                        .andThen(setCandleRedCommand()).andThen(Commands.waitSeconds(.25))
                        .andThen(setCandleYellowCommand()).andThen(Commands.waitSeconds(.35))
                        .andThen(setCandleRedCommand()).andThen(Commands.waitSeconds(.35))
                        .andThen(setCandleOrangeCommand()).andThen(Commands.waitSeconds(.25))
                        .andThen(setCandleGreenCommand()).andThen(Commands.waitSeconds(.4))
                        .andThen(setCandleOrangeCommand()).andThen(Commands.waitSeconds(.5))
                        .andThen(setCandleGreenCommand()).andThen(Commands.waitSeconds(.25))
                        .andThen(setCandleBlueCommand()).andThen(Commands.waitSeconds(.6))
                        .andThen(setCandlePurpleCommand()).andThen(Commands.waitSeconds(.8))));
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

    public void candleColorAndMode(Color color, CandleState state) {
        candleState = state;
        canColor = color;
    }

    public void stripColorAndMode(Color color, StripState state) {
        stripState = state;
        stripColor = color;
    }

    public void candleColor(Color color) {
        canColor = color;
    }

    public void stripColor(Color color) {
        stripColor = color;
    }

    public void candleMode(CandleState state) {
        candleState = state;
    }

    public void stripMode(StripState state) {
        stripState = state;
    }

    public void toggleCandle() {
        if (candleState == candleState.FLASH) {
            candleState = candleState.OFF;
        } else if (candleState == candleState.OFF) {
            candleState = candleState.FLASH;
        }
    }

    public void toggleStrip() {
        if (stripState == stripState.FLASH) {
            stripState = stripState.OFF;
        } else if (stripState == stripState.OFF) {
            stripState = stripState.FLASH;
        }
    }

    public void toggleCanColor() {
        if (canColor == Color.RED) {
            canColor = Color.ORANGE;
        } else if (canColor == Color.ORANGE) {
            canColor = Color.YELLOW;
        } else if (canColor == Color.YELLOW) {
            canColor = Color.GREEN;
        } else if (canColor == Color.GREEN) {
            canColor = Color.LBLUE;
        } else if (canColor == Color.LBLUE) {
            canColor = Color.BLUE;
        } else if (canColor == Color.BLUE) {
            canColor = Color.PURPLE;
        } else if (canColor == Color.PURPLE) {
            canColor = Color.PINK;
        } else if (canColor == Color.PINK) {
            canColor = Color.WHITE;
        } else if (canColor == Color.WHITE) {
            canColor = Color.OFF;
        } else if (canColor == Color.OFF) {
            canColor = Color.RED;
        }
    }

    public void toggleStripColor() {
        if (stripColor == Color.RED) {
            stripColor = Color.ORANGE;
        } else if (stripColor == Color.ORANGE) {
           stripColor = Color.YELLOW;
        } else if (stripColor == Color.YELLOW) {
            stripColor = Color.GREEN;
        } else if (stripColor == Color.GREEN) {
            stripColor = Color.LBLUE;
        } else if (stripColor == Color.LBLUE) {
            stripColor = Color.BLUE;
        } else if (stripColor == Color.BLUE) {
            stripColor = Color.PURPLE;
        } else if (stripColor == Color.PURPLE) {
            stripColor = Color.PINK;
        } else if (stripColor == Color.PINK) {
            stripColor = Color.WHITE;
        } else if (stripColor == Color.WHITE) {
            stripColor = Color.OFF;
        } else if (stripColor == Color.OFF) {
            stripColor = Color.RED;
        }
    }

    @Override
    public void periodic() {
        switch (candleState) {
            case FLASH:
                if (canflasher.get() >= .1) {
                    isCanOn = !isCanOn;
                    canflasher.reset();
                }
                if (canflasher.isRunning() == false) {
                    canflasher.start();
                }
                if (isCanOn) {
                    setLedsColor(0, 8, this.canColor.r, this.canColor.g, this.canColor.b);
                } else {
                    setLedsColor(0, 8, 0, 0, 0);
                }
                break;
            case OFF:
                setLedsColor(0, 8, 0, 0, 0);
                break;
            default:
                setLedsColor(0, 8, 0, 0, 0);
                break;
        }

        switch (stripState) {
            case FLASH:
                if (stripflasher.get() >= .1) {
                    isStripOn = !isStripOn;
                    stripflasher.reset();
                }
                if (stripflasher.isRunning() == false) {
                    stripflasher.start();
                }
                if (isStripOn) {
                    setLedsColor(8, 68, this.stripColor.r, this.stripColor.g, this.stripColor.b);
                } else {
                    setLedsColor(8, 68, 0, 0, 0);
                }
                break;
            case OFF:
                setLedsColor(8, 68, 0, 0, 0);
                break;
            default:
                setLedsColor(8, 68, 0, 0, 0);
                break;
        }
    }
}
