package frc.robot.subsystems.stick;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.util.MotorData;

public class WhipStickIOTalonFX implements WhipStickIO {
    private final TalonFX motor = new TalonFX(WhipStickConstants.TalonFX.CAN_ID,
            TunerConstants.DrivetrainConstants.CANBusName);
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> motorAmps = motor.getStatorCurrent();
    private final StatusSignal<AngularVelocity> vel = motor.getVelocity();

    private final DutyCycleOut vbusControl = new DutyCycleOut(0)
            .withEnableFOC(WhipStickConstants.TalonFX.USE_FOC);
    private final VoltageOut voltageControl = new VoltageOut(0)
            .withEnableFOC(WhipStickConstants.TalonFX.USE_FOC);

    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0);

    public WhipStickIOTalonFX() {
        motor.getConfigurator().apply(WhipStickConstants.TalonFX.CONFIG, 0.25);
        motor.getConfigurator().apply(WhipStickConstants.TalonFX.CURR_LIMITS, 0.25);

        BaseStatusSignal.setUpdateFrequencyForAll(50, motorVolts, motorAmps, vel);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(WhipStickIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorVolts, motorAmps);
        inputs.appliedVolts = motorVolts.getValueAsDouble();
        inputs.currentAmps = motorAmps.getValueAsDouble();
        inputs.motorData = MotorData.empty();
        inputs.isConnected = motor.isConnected();
    }

    @Override
    public void setVbus(double vBus) {
        motor.setControl(vbusControl.withOutput(vBus));
    }

    public double getVelocity() {
        return vel.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageControl.withOutput(volts));
    }

    public void setCurrent(double amps) {
        motor.setControl(currentControl.withOutput(amps));
    }
}
