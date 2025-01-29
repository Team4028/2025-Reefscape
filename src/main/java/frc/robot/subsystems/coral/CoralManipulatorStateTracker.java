package frc.robot.subsystems.coral;

public class CoralManipulatorStateTracker {
    public boolean hasCoral;
    public CoralStates state;

    public enum CoralStates {
        OFF,
        VBUS_FORWARD,
        VBUS_REVERSE,
        VOLTAGE_FORWARD,
        VOLTAGE_REVERSE,
    }

    public void setStateVBus(double vbus) {
        state = vbus > 0 ? CoralStates.VBUS_FORWARD : (vbus < 0 ? CoralStates.VBUS_REVERSE : CoralStates.OFF);
    }

    public void setStateVoltage(double volts) {
        state = volts > 0 ? CoralStates.VOLTAGE_FORWARD : (volts < 0 ? CoralStates.VOLTAGE_REVERSE : CoralStates.OFF);
    }
}
