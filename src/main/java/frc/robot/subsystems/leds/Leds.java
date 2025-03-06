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

    // Starting States
    private Color canColor = Color.WHITE;
    private Color stripColor = Color.WHITE;
    private Color stripColorTwo = Color.WHITE;
    private CandleState candleState = CandleState.OFF;
    private StripState stripState = StripState.OFF;
    private StripStateTwo stripStateTwo = StripStateTwo.OFF;

    // Booleans
    private boolean isCanOn = true;
    private boolean isStripOn = true;
    private boolean isStripTwoOn = true;
    private boolean inhale = true;
    private boolean inhaleTwo = true;
    private boolean seeAnyTag = false;
    private boolean rainbow = false;

    // Timers
    private Timer canFlasher;
    private Timer stripFlasher;
    private Timer stripFlasherTwo;

    // Tag Count
    private int aprilTagCount;

    // Index and Count
    private int canInd = 0;
    private int stripInd = 8;
    private int stripIndTwo = 38;
    private int canLength = 8;
    private int stripLength = 30;
    private int stripLengthTwo = 30;
    private int fullStripLength = 60;

    // Breathing and FOLLOWSTRIP
    private int canColorR = 0;
    private int canColorG = 0;
    private int canColorB = 0;
    private int stripRed = 0;
    private int stripGreen = 0;
    private int stripBlue = 0;
    private int stripTwoR = 0;
    private int stripTwoG = 0;
    private int stripTwoB = 0;
    private int breathPoint = 0;
    private int breathPointTwo = 0;

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
        LIME,
        SOLID,
        FOLLOWSTRIP,
        OFF;
    }

    public enum StripState {
        FLASH,
        BREATH,
        SOLID,
        OFF;
    }

    public enum StripStateTwo {
        FLASH,
        BREATH,
        SOLID,
        FOLLOWSTRIP,
        OFF;
    }

    public Leds() {
        candle = new CANdle(21, "rio");
        candle.configBrightnessScalar(.25);
        candle.configLEDType(LEDStripType.GRB);
        setCandleColor(Color.WHITE);
        setStripColor(Color.WHITE);
        canFlasher = new Timer();
        stripFlasher = new Timer();
        stripFlasherTwo = new Timer();
    }

    public void limelightToLeds(int aprilTagCount) {
        if (aprilTagCount == 1) {
            candleColorAndMode(Color.OFF, CandleState.LIME);
        } else if (aprilTagCount == 2) {
            candleColorAndMode(Color.RED, CandleState.LIME);
        } else if (aprilTagCount == 3) {
            candleColorAndMode(Color.PURPLE, CandleState.LIME);
        } else if (aprilTagCount == 4) {
            candleColorAndMode(Color.BLUE, CandleState.LIME);
        } else if (aprilTagCount > 4) {
            candleColorAndMode(Color.WHITE, CandleState.LIME);
        } else {
            candleMode(CandleState.OFF);
        }
    }

    public Command limelightToLedsCommand(int aprilTagCount) {
        return Commands.runOnce(() -> limelightToLeds(aprilTagCount));
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
        candle.setLEDs(this.color.r, this.color.g, this.color.b, 0, canInd, canLength);
    }

    public void setStripColor(Color color) {
        this.color = color;
        candle.animate(null);
        candle.setLEDs(this.color.r, this.color.g, this.color.b, 0, stripInd, fullStripLength);
    }

    // public Command sevenAnimationCandle() {
    // return Commands.repeatingSequence(setCandleRedCommand()
    // .andThen(Commands.waitSeconds(.65).andThen(setNoColorCandle()).andThen(Commands.waitSeconds(.05))
    // .andThen(setCandleRedCommand()).andThen(Commands.waitSeconds(.25))
    // .andThen(setCandleYellowCommand())
    // .andThen(Commands.waitSeconds(.35))
    // .andThen(setCandleRedCommand()).andThen(Commands.waitSeconds(.35))
    // .andThen(setCandleOrangeCommand()).andThen(Commands.waitSeconds(.25))
    // .andThen(setCandleGreenCommand()).andThen(Commands.waitSeconds(.9))
    // .andThen(setCandleBlueCommand()).andThen(Commands.waitSeconds(1)).andThen(setNoColorCandle())
    // .andThen(Commands.waitSeconds(.4))
    // .andThen(setCandleRedCommand())
    // .andThen(Commands.waitSeconds(.65)).andThen(setNoColorCandle())
    // .andThen(Commands.waitSeconds(.05))
    // .andThen(setCandleRedCommand()).andThen(Commands.waitSeconds(.25))
    // .andThen(setCandleYellowCommand()).andThen(Commands.waitSeconds(.35))
    // .andThen(setCandleRedCommand()).andThen(Commands.waitSeconds(.35))
    // .andThen(setCandleOrangeCommand()).andThen(Commands.waitSeconds(.25))
    // .andThen(setCandleGreenCommand()).andThen(Commands.waitSeconds(.4))
    // .andThen(setCandleOrangeCommand()).andThen(Commands.waitSeconds(.5))
    // .andThen(setCandleGreenCommand()).andThen(Commands.waitSeconds(.25))
    // .andThen(setCandleBlueCommand()).andThen(Commands.waitSeconds(.6))
    // .andThen(setCandlePurpleCommand()).andThen(Commands.waitSeconds(.8))));
    // }

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

    public Command candleColorAndModeCommand(Color color, CandleState state) {
        return Commands.runOnce(() -> candleColorAndMode(color, state));
    }

    public void stripColorAndMode(Color color, StripState state) {
        stripState = state;
        stripColor = color;
    }

    public Command stripColorAndModeCommand(Color color, StripState state) {
        return Commands.runOnce(() -> stripColorAndMode(color, state));
    }

    public void stripTwoColorAndMode(Color color, StripStateTwo state) {
        stripStateTwo = state;
        stripColorTwo = color;
    }

    public Command stripTwoColorAndModeCommand(Color color, StripStateTwo state) {
        return Commands.runOnce(() -> stripTwoColorAndMode(color, state));
    }

    public void candleColor(Color color) {
        canColor = color;
    }

    public Command candleColorCommand(Color color) {
        return Commands.runOnce(() -> candleColor(color));
    }

    public void stripColor(Color color) {
        stripColor = color;
    }

    public Command stripColorCommand(Color color) {
        return Commands.runOnce(() -> stripColor(color));
    }

    public void stripTwoColor(Color color) {
        stripColorTwo = color;
    }

    public Command stripTwoColorCommand(Color color) {
        return Commands.runOnce(() -> stripTwoColor(color));
    }

    public void candleMode(CandleState state) {
        candleState = state;
    }

    public Command candleModeCommand(CandleState state) {
        return Commands.runOnce(() -> candleMode(state));
    }

    public void stripMode(StripState state) {
        stripState = state;
    }

    public Command stripModeCommand(StripState state) {
        return runOnce(() -> stripMode(state));
    }

    public void stripTwoMode(StripStateTwo state) {
        stripStateTwo = state;
    }

    public Command stripTwoModeCommand(StripStateTwo state) {
        return Commands.runOnce(() -> stripTwoMode(state));
    }

    public void toggleCandle() {
        if (candleState == CandleState.FLASH) {
            candleState = CandleState.OFF;
        } else if (candleState == CandleState.OFF) {
            candleState = CandleState.FLASH;
        }
    }

    public void toggleStrip() {
        if (stripState == StripState.FLASH) {
            stripState = StripState.OFF;
        } else if (stripState == StripState.OFF) {
            stripState = StripState.FLASH;
        }
    }

    // public void toggleCanColor() {
    //     if (canColor == Color.RED) {
    //         canColor = Color.ORANGE;
    //     } else if (canColor == Color.ORANGE) {
    //         canColor = Color.YELLOW;
    //     } else if (canColor == Color.YELLOW) {
    //         canColor = Color.GREEN;
    //     } else if (canColor == Color.GREEN) {
    //         canColor = Color.LBLUE;
    //     } else if (canColor == Color.LBLUE) {
    //         canColor = Color.BLUE;
    //     } else if (canColor == Color.BLUE) {
    //         canColor = Color.PURPLE;
    //     } else if (canColor == Color.PURPLE) {
    //         canColor = Color.PINK;
    //     } else if (canColor == Color.PINK) {
    //         canColor = Color.WHITE;
    //     } else if (canColor == Color.WHITE) {
    //         canColor = Color.OFF;
    //     } else if (canColor == Color.OFF) {
    //         canColor = Color.RED;
    //     }
    // }

    public void toggleStripColor() {
        switch (stripColor) {
            case RED -> stripColor = Color.ORANGE;
            case ORANGE -> stripColor = Color.YELLOW;
            case YELLOW -> stripColor = Color.GREEN;
            case GREEN -> stripColor = Color.LBLUE;
            case LBLUE -> stripColor = Color.BLUE;
            case BLUE -> stripColor = Color.PURPLE;
            case PURPLE -> stripColor = Color.PINK;
            case PINK -> stripColor = Color.WHITE;
            case WHITE -> stripColor = Color.RED;
            default -> {
            }
        }
    }

    public int toggleAprilTags() {
        seeAnyTag = true;
        aprilTagCount = (aprilTagCount + 1) % 6;
        return aprilTagCount;
    }

    public void offTag() {
        seeAnyTag = false;
    }

    public Command offTagCommand() {
        return Commands.runOnce(() -> offTag());
    }

    public void toggleRainbow() {
        rainbow = !rainbow;
    }

    @Override
    public void periodic() {
        switch (candleState) {
            case FLASH:
                canColorR = this.canColor.r;
                canColorG = this.canColor.g;
                canColorB = this.canColor.b;
                if (canFlasher.get() >= .1) {
                    isCanOn = !isCanOn;
                    canFlasher.reset();
                }
                if (canFlasher.isRunning() == false) {
                    canFlasher.start();
                }
                if (isCanOn) {
                    setLedsColor(canInd, canLength, canColorR, canColorG, canColorB);
                } else {
                    setLedsColor(canInd, canLength, 0, 0, 0);
                }
                break;
            case LIME:
                canColorR = this.canColor.r;
                canColorG = this.canColor.g;
                canColorB = this.canColor.b;
                if (canFlasher.get() >= .15) {
                    isCanOn = !isCanOn;
                    canFlasher.reset();
                }
                if (canFlasher.isRunning() == false) {
                    canFlasher.start();
                }
                if (isCanOn) {
                    setLedsColor(canInd, canLength, canColorR, canColorG, canColorB);
                } else {
                    setLedsColor(canInd, canLength, 0, 110, 0);
                }
                break;
            case SOLID:
                canColorR = this.canColor.r;
                canColorG = this.canColor.g;
                canColorB = this.canColor.b;
                setLedsColor(canInd, canLength, canColorR, canColorG, canColorB);
                break;
            case FOLLOWSTRIP:
                setLedsColor(canInd, canLength, stripRed, stripGreen, stripBlue);
                break;
            case OFF:
                setLedsColor(canInd, canLength, 0, 0, 0);
                break;
            default:
                setLedsColor(canInd, canLength, 0, 0, 0);
                break;
        }

        switch (stripState) {
            case FLASH:
                stripRed = this.stripColor.r;
                stripGreen = this.stripColor.g;
                stripBlue = this.stripColor.b;
                if (stripFlasher.get() >= .1) {
                    isStripOn = !isStripOn;
                    stripFlasher.reset();
                }
                if (stripFlasher.isRunning() == false) {
                    stripFlasher.start();
                }
                if (isStripOn) {
                    setLedsColor(stripInd, stripLength, stripRed, stripGreen, stripBlue);
                } else {
                    setLedsColor(stripInd, stripLength, 0, 0, 0);
                }
                break;
            case BREATH:
                stripRed = Math.round((this.stripColor.r * breathPoint) / 45);
                stripGreen = Math.round((this.stripColor.g * breathPoint) / 45);
                stripBlue = Math.round((this.stripColor.b * breathPoint) / 45);
                if (inhale) {
                    if (breathPoint <= 45) {
                        setLedsColor(stripInd, stripLength, stripRed, stripGreen, stripBlue);
                        breathPoint += 1;
                    } else {
                        inhale = false;
                    }
                } else {
                    if (breathPoint > 0) {
                        setLedsColor(stripInd, stripLength, stripRed, stripGreen, stripBlue);
                        breathPoint -= 1;
                    }
                    if (breathPoint == 0) {
                        inhale = true;
                    }
                }
                if (rainbow && breathPoint == 0) {
                    toggleStripColor();
                }
                break;
            case SOLID:
                stripRed = this.stripColor.r;
                stripGreen = this.stripColor.g;
                stripBlue = this.stripColor.b;
                setLedsColor(stripInd, stripLength, stripRed, stripGreen, stripBlue);
                break;
            case OFF:
                setLedsColor(stripInd, stripLength, 0, 0, 0);
                break;
            default:
                setLedsColor(stripInd, stripLength, 0, 0, 0);
                break;
        }

        switch (stripStateTwo) {
            case FLASH:
                stripTwoR = this.stripColorTwo.r;
                stripTwoG = this.stripColorTwo.g;
                stripTwoB = this.stripColorTwo.b;
                if (stripFlasherTwo.get() >= .1) {
                    isStripTwoOn = !isStripTwoOn;
                    stripFlasherTwo.reset();
                }
                if (stripFlasherTwo.isRunning() == false) {
                    stripFlasherTwo.start();
                }
                if (isStripTwoOn) {
                    setLedsColor(stripIndTwo, stripLengthTwo, this.stripColorTwo.r, this.stripColorTwo.g,
                            this.stripColorTwo.b);
                } else {
                    setLedsColor(stripIndTwo, stripLengthTwo, 0, 0, 0);
                }
                break;
            case BREATH:
                stripTwoR = Math.round((this.stripColorTwo.r * breathPointTwo) / 45);
                stripTwoG = Math.round((this.stripColorTwo.g * breathPointTwo) / 45);
                stripTwoB = Math.round((this.stripColorTwo.b * breathPointTwo) / 45);
                if (inhaleTwo) {
                    if (breathPointTwo <= 45) {
                        setLedsColor(stripIndTwo, stripLengthTwo, stripTwoR, stripTwoG, stripTwoB);
                        breathPointTwo += 1;
                    } else {
                        inhaleTwo = false;
                    }
                } else {
                    if (breathPointTwo >= 0) {
                        setLedsColor(stripIndTwo, stripLengthTwo, stripTwoR, stripTwoG, stripTwoB);
                        breathPointTwo -= 1;
                    } else {
                        inhaleTwo = true;
                    }
                }
                break;
            case FOLLOWSTRIP:
                setLedsColor(stripIndTwo, stripLengthTwo, stripRed, stripGreen, stripBlue);
                break;
            case SOLID:
                stripTwoR = this.stripColorTwo.r;
                stripTwoG = this.stripColorTwo.g;
                stripTwoB = this.stripColorTwo.b;
                setLedsColor(stripIndTwo, stripLengthTwo, stripTwoR, stripTwoG,
                        stripTwoB);
                break;
            case OFF:
                setLedsColor(stripIndTwo, stripLengthTwo, 0, 0, 0);
                break;
            default:
                setLedsColor(stripIndTwo, stripLengthTwo, 0, 0, 0);
                break;
        }
    }
}
