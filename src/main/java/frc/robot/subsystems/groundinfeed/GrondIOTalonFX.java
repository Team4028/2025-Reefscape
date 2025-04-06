package frc.robot.subsystems.groundinfeed;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.util.MotorData;

public class GrondIOTalonFX implements GrondIO {
    private final TalonFX motor = new TalonFX(GrondConstants.TalonFX.CAN_ID, TunerConstants.DrivetrainConstants.CANBusName);
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> motorCurrent = motor.getStatorCurrent();

    private final DutyCycleOut vbusControl = new DutyCycleOut(0).withEnableFOC(GrondConstants.TalonFX.USE_FOC);
    private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(GrondConstants.TalonFX.USE_FOC);

    public GrondIOTalonFX() {
        motor.getConfigurator().apply(GrondConstants.TalonFX.motorConfigs);
        motor.getConfigurator().apply(GrondConstants.TalonFX.currLimits);

        BaseStatusSignal.setUpdateFrequencyForAll(100, motorVolts, motorCurrent);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GrondIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorVolts, motorCurrent);
        inputs.appliedVoltage = motorVolts.getValueAsDouble();
        inputs.currentAmps = motorCurrent.getValueAsDouble();
        inputs.motorData = MotorData.empty();
    }

    @Override
    public void setVbus(double vbus) {
        motor.setControl(vbusControl.withOutput(vbus));
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltageControl.withOutput(voltage));
    }
}
