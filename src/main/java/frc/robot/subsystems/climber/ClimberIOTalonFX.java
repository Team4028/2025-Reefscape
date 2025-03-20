package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.util.MotorData;

public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX motor = new TalonFX(ClimberConstants.TalonFX.MOTOR_ID, TunerConstants.DrivetrainConstants.CANBusName);
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> motorAmps = motor.getStatorCurrent();
    private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(ClimberConstants.USE_FOC);
    private final DutyCycleOut vbusControl = new DutyCycleOut(0).withEnableFOC(ClimberConstants.USE_FOC);

    public ClimberIOTalonFX() {
        motor.getConfigurator().apply(ClimberConstants.TalonFX.currentLimitConfigs, 0.25);

        BaseStatusSignal.setUpdateFrequencyForAll(100, motorAmps, motorVolts);
        motor.optimizeBusUtilization();
    }

   @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorAmps, motorVolts);
        inputs.appliedVoltage = motorVolts.getValueAsDouble();
        inputs.currentAmps = motorAmps.getValueAsDouble();
        inputs.motorData = MotorData.empty();
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
