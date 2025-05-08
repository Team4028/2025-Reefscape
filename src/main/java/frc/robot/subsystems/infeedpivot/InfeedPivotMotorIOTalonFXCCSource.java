package frc.robot.subsystems.infeedpivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmConstants;

public class InfeedPivotMotorIOTalonFXCCSource implements InfeedPivotMotorIO {
    private final TalonFX motor = new TalonFX(InfeedPivotConstants.TalonFXCC.CAN_ID);
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> motorCurrent = motor.getSupplyCurrent();
    private final StatusSignal<Angle> motorPosition = motor.getPosition();
    private final StatusSignal<AngularVelocity> motorVel = motor.getVelocity();
    private final DutyCycleOut vbusControl = new DutyCycleOut(0).withEnableFOC(InfeedPivotConstants.TalonFX.USE_FOC);
    private final VoltageOut voltControl = new VoltageOut(0).withEnableFOC(InfeedPivotConstants.TalonFX.USE_FOC);
    private final MotionMagicVoltage pid = new MotionMagicVoltage(0);

    public InfeedPivotMotorIOTalonFXCCSource() {
        motor.getConfigurator().apply(InfeedPivotConstants.TalonFXCC.motorConfigs);
        motor.getConfigurator().apply(InfeedPivotConstants.TalonFXCC.feedbackConfigs);
        motor.getConfigurator().apply(InfeedPivotConstants.TalonFXCC.softLimits);
        motor.getConfigurator().apply(InfeedPivotConstants.TalonFXCC.mmConfigs);
        motor.getConfigurator().apply(InfeedPivotConstants.TalonFXCC.pidConfigs);
        BaseStatusSignal.setUpdateFrequencyForAll(50, motorVolts, motorCurrent, motorPosition, motorVel);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(InfeedPivotIOMotorInputs inputs) {
        BaseStatusSignal.refreshAll(motorPosition, motorVel, motorVolts, motorCurrent);
        inputs.isConnected = motor.isConnected();
        inputs.currentA = motorCurrent.getValueAsDouble();
        inputs.appliedV = motorVolts.getValueAsDouble();
        inputs.velRad = motorVel.getValueAsDouble() * ArmConstants.PI_2;
        inputs.positionRad = motorPosition.getValueAsDouble() * ArmConstants.PI_2;
        InfeedPivotMotorIO.super.updateInputs(inputs);
    }

    @Override
    public void setVBus(double vbus) {
        if (!Constants.CHAR_MODE)
            motor.setControl(vbusControl.withOutput(vbus));
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltControl.withOutput(voltage));
    }

    @Override
    public void setPid(double posRad) {
        if (Constants.CHAR_MODE) return;
        motor.setControl(pid.withPosition(posRad / ArmConstants.PI_2));
    }
}
