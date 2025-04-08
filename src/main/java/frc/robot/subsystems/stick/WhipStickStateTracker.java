package frc.robot.subsystems.stick;

public class WhipStickStateTracker {
    public boolean hasGP;
    public WhipStickStates state;

    public WhipStickStateTracker() {
        state = WhipStickStates.OFF;
        hasGP = false;
    }

    public void setStateVBus(double vbus) {
        state = vbus > 0 ? WhipStickStates.VBUS_FORWARD
                : (vbus < 0 ? WhipStickStates.VBUS_REVERSE : WhipStickStates.OFF);
    }
}
