package frc.robot.subsystems.stick;

public class WhipStickStateTracker {
    public boolean hasCoral;
    public WhipStickStates state;

    public WhipStickStateTracker() {
        state = WhipStickStates.OFF;
        hasCoral = false;
    }

    public void setStateVBus(double vbus) {
        state = vbus > 0 ? WhipStickStates.VBUS_FORWARD
                : (vbus < 0 ? WhipStickStates.VBUS_REVERSE : WhipStickStates.OFF);
    }
}
