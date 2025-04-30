package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.util.MathUtils;

public class ElevatorStateTracker {

    // Change values later for realzies
    public static final class ElevatorPositions {
        public enum Reef {
            L1(15), L2(30), L3(45), L4(50), HOLD(20), OFF(0);

            public final double position;

            Reef(double position) {
                this.position = position;
            }
        }
    }

    @AutoLogOutput
    public ElevatorStates state;
    public ElevatorPositions.Reef reefState;
    private int reefCount;

    public void cycleReefState() {
        reefState = switch (reefState) {
            case L1 -> ElevatorPositions.Reef.L2;
            case L2 -> ElevatorPositions.Reef.L3;
            case L3 -> ElevatorPositions.Reef.L4;
            case L4 -> ElevatorPositions.Reef.L1;
            default -> reefState;
        };
    }

    public void shiftReefCount(int shift) {
        reefCount = MathUtils.clamp(reefCount + shift, 0, ElevatorPositions.Reef.values().length);
    }

    public void applyReefCount() {
        reefState = ElevatorPositions.Reef.values()[reefCount];
    }

    public ElevatorStateTracker() {
        state = ElevatorStates.OFF;
        reefState = ElevatorPositions.Reef.OFF;
        reefCount = 0;
    }

    public void setStateVBus(double vbus) {
        state = vbus > 0 ? ElevatorStates.VBUS_FORWARD : (vbus < 0 ? ElevatorStates.VBUS_REVERSE : ElevatorStates.OFF);
    }

    public void setStateVoltage(double volts) {
        state = volts > 0 ? ElevatorStates.VOLTAGE_FORWARD : (volts < 0 ? ElevatorStates.VOLTAGE_REVERSE : ElevatorStates.OFF);
    }
}
