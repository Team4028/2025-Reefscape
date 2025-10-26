package frc.robot.subsystems.cage;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.util.MotorData;

public class CageIOTalonFX implements CageIO {
    private final TalonFX motor = new TalonFX(CageConstants.TalonFX.MOTOR_ID);
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> motorAmps = motor.getStatorCurrent();
    private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(CageConstants.USE_FOC);
    private final DutyCycleOut vbusControl = new DutyCycleOut(0).withEnableFOC(CageConstants.USE_FOC);

    public CageIOTalonFX() {
        motor.getConfigurator().apply(CageConstants.TalonFX.currentLimitConfigs, 0.25);
        motor.getConfigurator().apply(CageConstants.TalonFX.motorConfigs, 0.25);

        BaseStatusSignal.setUpdateFrequencyForAll(50, motorAmps, motorVolts);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(CageIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorAmps, motorVolts);
        inputs.appliedVoltage = motorVolts.getValueAsDouble();
        inputs.currentAmps = motorAmps.getValueAsDouble();
        inputs.motorData = MotorData.empty();
        inputs.isConnected = motor.isConnected();
    }

    @Override
    public void setVbus(double vBus) {
        motor.setControl(vbusControl.withOutput(vBus));
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageControl.withOutput(volts));
    }
}
