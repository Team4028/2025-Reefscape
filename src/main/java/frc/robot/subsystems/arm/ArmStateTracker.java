package frc.robot.subsystems.arm;

import java.util.function.Consumer;

import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.subsystems.arm.ArmConstants.ArmSafetyData;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.MathUtil;

public class ArmStateTracker {

    public ArmStates state;

    public ArmStateTracker() {
        state = ArmStates.OFF;
    }

    public void setStateVBus(double vbus) {
        state = vbus > 0 ? ArmStates.VBUS_FORWARD : (vbus < 0 ? ArmStates.VBUS_REVERSE : ArmStates.OFF);
    }

    public void setStateVoltage(double volts) {
        state = volts > 0 ? ArmStates.VOLTAGE_FORWARD
                : (volts < 0 ? ArmStates.VOLTAGE_REVERSE : ArmStates.OFF);
    }
}
