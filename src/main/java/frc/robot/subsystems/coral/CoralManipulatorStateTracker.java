package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CoralManipulatorStateTracker {
    public boolean hasCoral;
    public CoralManipulatorStates state;
    boolean coralInfeeding;
    boolean coralOutfeeding;

    public CoralManipulatorStateTracker() {
        state = CoralManipulatorStates.OFF;
        hasCoral = false;
    }

    public void setStateVBus(double vbus) {
        state = vbus > 0 ? CoralManipulatorStates.VBUS_FORWARD
                : (vbus < 0 ? CoralManipulatorStates.VBUS_REVERSE : CoralManipulatorStates.OFF);
        if (vbus > 0) {
            coralInfeeding = true;
        } else {
            if (vbus < 0) {
                coralOutfeeding = true;
            } else {
                coralInfeeding = false;
                coralOutfeeding = false;
            }
        }
    }

    public void setStateVoltage(double volts) {
        state = volts > 0 ? CoralManipulatorStates.VOLTAGE_FORWARD
                : (volts < 0 ? CoralManipulatorStates.VOLTAGE_REVERSE : CoralManipulatorStates.OFF);
    }

    public Trigger outfeeding() {
        return new Trigger(() -> coralOutfeeding);
    }

    public Trigger infeeding() {
        return new Trigger(() -> coralInfeeding);
    }

    public void toggleFeed() {
        if (coralOutfeeding == false) {
            if (coralInfeeding == false) {
                coralOutfeeding = true;
            } else {
                coralInfeeding = false;
                coralOutfeeding = false;
            }
        } else {
            coralOutfeeding = false;
            coralInfeeding = true;
        }
    }
}
