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
}
