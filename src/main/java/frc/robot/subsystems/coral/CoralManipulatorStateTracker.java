package frc.robot.subsystems.coral;

public class CoralManipulatorStateTracker {
    public boolean hasCoral;
    public CoralManipulatorStates state;

    public CoralManipulatorStateTracker() {
        state = CoralManipulatorStates.OFF;
        hasCoral = false;
    }

    public void setStateVBus(double vbus) {
        state = vbus > 0 ? CoralManipulatorStates.VBUS_FORWARD
                : (vbus < 0 ? CoralManipulatorStates.VBUS_REVERSE : CoralManipulatorStates.OFF);
    }

    public void setStateVoltage(double volts) {
        state = volts > 0 ? CoralManipulatorStates.VOLTAGE_FORWARD
                : (volts < 0 ? CoralManipulatorStates.VOLTAGE_REVERSE : CoralManipulatorStates.OFF);
    }
}
