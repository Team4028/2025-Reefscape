package frc.robot.subsystems.coral;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class CoralManipulatorIOTalonFX implements CoralManipulatorIO {
    private final TalonFX motor = new TalonFX(CoralManipulatorConstants.TalonFX.CAN_ID);
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> motorAmps = motor.getSupplyCurrent();

    private final DutyCycleOut vbusControl = new DutyCycleOut(0)
            .withEnableFOC(CoralManipulatorConstants.TalonFX.USE_FOC);
    private final VoltageOut voltageControl = new VoltageOut(0)
            .withEnableFOC(CoralManipulatorConstants.TalonFX.USE_FOC);

    public CoralManipulatorIOTalonFX() {
        motor.getConfigurator().apply(CoralManipulatorConstants.TalonFX.CONFIG);
    }

    @Override
    public void updateInputs(CoralManipulatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorVolts, motorAmps);

        inputs.appliedVolts = motorVolts.getValueAsDouble();
        inputs.currentAmps = motorAmps.getValueAsDouble();
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
