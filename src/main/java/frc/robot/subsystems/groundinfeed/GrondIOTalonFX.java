package frc.robot.subsystems.groundinfeed;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.util.MotorData;

public class GrondIOTalonFX implements GrondIO {
    private final TalonFX motor;
    private final StatusSignal<Voltage> motorVolts;
    private final StatusSignal<Current> motorCurrent;

    private final DutyCycleOut vbusControl;
    private final VoltageOut voltageControl;
    private final TorqueCurrentFOC currentControl;

    public GrondIOTalonFX(boolean isLeft) {
        motor = new TalonFX(isLeft ? GrondConstants.TalonFX.CAN_ID_LEFT : GrondConstants.TalonFX.CAN_ID_RIGHT,
                TunerConstants.DrivetrainConstants.CANBusName);
        motor.getConfigurator()
                .apply(isLeft ? GrondConstants.TalonFX.motorConfigs : GrondConstants.TalonFX.motorConfigsRight);
        motor.getConfigurator().apply(GrondConstants.TalonFX.currLimits);

        motorVolts = motor.getMotorVoltage();
        motorCurrent = motor.getStatorCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(50, motorVolts, motorCurrent);
        motor.optimizeBusUtilization();
        vbusControl = new DutyCycleOut(0).withEnableFOC(GrondConstants.TalonFX.USE_FOC);
        voltageControl = new VoltageOut(0).withEnableFOC(GrondConstants.TalonFX.USE_FOC);
        currentControl = new TorqueCurrentFOC(0);
    }

    public void setBrakeMode(boolean isBrake) {
        motor.getConfigurator().apply(GrondConstants.TalonFX.motorConfigs
                .withNeutralMode(isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast));
    }

    @Override
    public void updateInputs(GrondIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorVolts, motorCurrent);
        inputs.appliedVoltage = motorVolts.getValueAsDouble();
        inputs.currentAmps = motorCurrent.getValueAsDouble();
        inputs.motorData = MotorData.empty();
        inputs.isConnected = motor.isConnected();
    }

    @Override
    public void setVbus(double vbus) {
        motor.setControl(vbusControl.withOutput(vbus));
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltageControl.withOutput(voltage));
    }

    @Override
    public void setCurrent(double amps) {
        motor.setControl(currentControl.withOutput(amps));
    }
}
