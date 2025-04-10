package frc.robot.subsystems.infeedpivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.ArmConstants;

public class InfeedPivotMotorIOTalonFX implements InfeedPivotMotorIO {
    private final TalonFX motor = new TalonFX(InfeedPivotConstants.TalonFX.CAN_ID, TunerConstants.DrivetrainConstants.CANBusName);
    private final StatusSignal<Angle> position = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> motorCurrent = motor.getStatorCurrent();

    private final DutyCycleOut vbusControl = new DutyCycleOut(0).withEnableFOC(InfeedPivotConstants.TalonFX.USE_FOC);
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0).withEnableFOC(InfeedPivotConstants.TalonFX.USE_FOC).withSlot(0);

    public InfeedPivotMotorIOTalonFX() {
        motor.getConfigurator().apply(InfeedPivotConstants.TalonFX.motorConfigs);
        motor.getConfigurator().apply(InfeedPivotConstants.TalonFX.currLimits);
        motor.getConfigurator().apply(InfeedPivotConstants.TalonFX.pidConfigs);
        motor.getConfigurator().apply(InfeedPivotConstants.TalonFX.mmConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(100, position, velocity, motorVolts, motorCurrent);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(InfeedPivotIOMotorInputs inputs) {
        inputs.positionRad = position.getValueAsDouble() * ArmConstants.PI_2;
        inputs.velRad = velocity.getValueAsDouble() * ArmConstants.PI_2;
        inputs.appliedV = motorVolts.getValueAsDouble();
        inputs.currentA = motorCurrent.getValueAsDouble();
        InfeedPivotMotorIO.super.updateInputs(inputs);
    }

    @Override
    public void setVBus(double vbus) {
        motor.setControl(vbusControl.withOutput(vbus));
    }

    @Override
    public void setPid(double posRad) {
        motor.setControl(positionControl.withPosition(posRad / ArmConstants.PI_2));
    }

    @Override
    public void zeroPosition(double zeroPosRad) {
        motor.setPosition(zeroPosRad / ArmConstants.PI_2);
    }
}
