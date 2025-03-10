package frc.robot.subsystems.algae;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.climber.ClimberConstants;

public class AlgaeManipulatorIOTalonFX implements AlgaeManipulatorIO {
    private final TalonFX motor = new TalonFX(AlgaeManipulatorConstants.TalonFX.CAN_ID);;
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> motorAmps = motor.getStatorCurrent();
    private final DutyCycleOut vbusControl = new DutyCycleOut(0).withEnableFOC(ClimberConstants.USE_FOC);

    public AlgaeManipulatorIOTalonFX() {
        BaseStatusSignal.setUpdateFrequencyForAll(20, motorVolts, motorAmps);
        motor.optimizeBusUtilization();
        motor.getConfigurator().apply(AlgaeManipulatorConstants.TalonFX.motorConfigs);
        motor.getConfigurator().apply(AlgaeManipulatorConstants.TalonFX.currentConfigs);
    }
    @Override
    public void updateInputs(AlgaeManipulatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorAmps, motorVolts);
        inputs.appliedVolts = motorVolts.getValueAsDouble();
        inputs.currentAmps = motorAmps.getValueAsDouble();
    }
    @Override
    public void setVbus(double vbus) {
        motor.setControl(vbusControl.withOutput(vbus));
    }
    @Override
    public void setVoltage(double volts) {
        setVbus(volts / RobotController.getBatteryVoltage());
    }
}
