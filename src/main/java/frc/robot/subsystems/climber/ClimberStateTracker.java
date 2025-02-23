package frc.robot.subsystems.climber;



public class ClimberStateTracker {
    public ClimberStates state;

    public ClimberStateTracker() {
        state = ClimberStates.OFF;
    }

    public void setStateVBus(double vbus) {
        state = vbus > 0 ? ClimberStates.VBUS_FORWARD : (vbus < 0 ? ClimberStates.VBUS_REVERSE : ClimberStates.OFF);
    }

    public void setStateVoltage(double volts) {
        state = volts > 0 ? ClimberStates.VOLTAGE_FORWARD
                : (volts < 0 ? ClimberStates.VOLTAGE_REVERSE : ClimberStates.OFF);
    }
}
