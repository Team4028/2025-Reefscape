package frc.robot.subsystems.singulator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.util.MotorData;

public class SingulatorIOTalonFX implements SingulatorIO {
    private final TalonFX motor = new TalonFX(SingulatorConstants.TalonFX.CAN_ID,
            TunerConstants.DrivetrainConstants.CANBusName);
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> motorAmps = motor.getStatorCurrent();

    private final DutyCycleOut vbusControl = new DutyCycleOut(0).withEnableFOC(SingulatorConstants.TalonFX.USE_FOC);
    private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(SingulatorConstants.TalonFX.USE_FOC);

    public SingulatorIOTalonFX() {
        motor.getConfigurator().apply(SingulatorConstants.TalonFX.motorConfigs);
        motor.getConfigurator().apply(SingulatorConstants.TalonFX.currLims);
        BaseStatusSignal.setUpdateFrequencyForAll(100, motorVolts, motorAmps);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(SingulatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorVolts, motorAmps);
        inputs.appliedVolts = motorVolts.getValueAsDouble();
        inputs.currentAmps = motorAmps.getValueAsDouble();
        inputs.motorData = MotorData.empty();
    }

    @Override
    public void setVBus(double vbus) {
        motor.setControl(vbusControl.withOutput(vbus));
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltageControl.withOutput(voltage));
    }
}
