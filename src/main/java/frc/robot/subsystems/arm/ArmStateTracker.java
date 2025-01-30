package frc.robot.subsystems.arm;

import java.util.function.Consumer;

import frc.robot.subsystems.arm.ArmConstants.ArmSafetyData;
import frc.robot.util.MathUtil;

public class ArmStateTracker {

    public ArmStates state;
    private boolean isInDanger;

    public ArmStateTracker() {
        state = ArmStates.OFF;
        isInDanger = true;
    }

    public void setInDanger(boolean isInDanger, Consumer<ArmSafetyData> pidContinuousOutputUpdateHandler) {
        this.isInDanger = isInDanger;
        pidContinuousOutputUpdateHandler.accept(getArmSafety());
    }

    public double safeClampRange(double value) {
        var range = getArmSafety().range();
        return MathUtil.clamp(value, range[0], range[1]);
    }

    public ArmSafetyData getArmSafety() {
        return isInDanger ? ArmConstants.SAFETY_RANGE : ArmConstants.UNSAFE_RANGE;
    }

    public void setStateVBus(double vbus) {
        state = vbus > 0 ? ArmStates.VBUS_FORWARD : (vbus < 0 ? ArmStates.VBUS_REVERSE : ArmStates.OFF);
    }

    public void setStateVoltage(double volts) {
        state = volts > 0 ? ArmStates.VOLTAGE_FORWARD
                : (volts < 0 ? ArmStates.VOLTAGE_REVERSE : ArmStates.OFF);
    }
}
