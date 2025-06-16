package frc.robot.subsystems.cage;



public class CageStateTracker {
    public CageStates state;

    public CageStateTracker() {
        state = CageStates.OFF;
    }

    public void setStateVBus(double vbus) {
        state = vbus > 0 ? CageStates.VBUS_FORWARD : (vbus < 0 ? CageStates.VBUS_REVERSE : CageStates.OFF);
    }

    public void setStateVoltage(double volts) {
        state = volts > 0 ? CageStates.VOLTAGE_FORWARD
                : (volts < 0 ? CageStates.VOLTAGE_REVERSE : CageStates.OFF);
    }
}
