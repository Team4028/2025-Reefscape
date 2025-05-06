package frc.robot.subsystems.infeedpivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.ArmConstants;

public class InfeedPivotMotorIOTalonFX implements InfeedPivotMotorIO {
    private final TalonFX motor = new TalonFX(InfeedPivotConstants.TalonFX.CAN_ID,
            TunerConstants.DrivetrainConstants.CANBusName);
    private final StatusSignal<Angle> position = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> motorCurrent = motor.getStatorCurrent();

    private final DutyCycleOut vbusControl = new DutyCycleOut(0).withEnableFOC(InfeedPivotConstants.TalonFX.USE_FOC);
    private final VoltageOut voltControl = new VoltageOut(0).withEnableFOC(InfeedPivotConstants.TalonFX.USE_FOC);
    private final ProfiledPIDController pid = InfeedPivotConstants.pidConfig.makeProfiledPIDController();
    private final ArmFeedforward aFF = InfeedPivotConstants.pidConfig.makeArmFeedforward();

    public InfeedPivotMotorIOTalonFX() {
        motor.getConfigurator().apply(InfeedPivotConstants.TalonFX.motorConfigs);
        motor.getConfigurator().apply(InfeedPivotConstants.TalonFX.currLimits);
        motor.getConfigurator().apply(InfeedPivotConstants.TalonFX.softLimits);

        BaseStatusSignal.setUpdateFrequencyForAll(50, position, velocity, motorVolts, motorCurrent);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(InfeedPivotIOMotorInputs inputs) {
        BaseStatusSignal.refreshAll(position, velocity, motorVolts, motorCurrent);
        inputs.positionRad = position.getValueAsDouble() * ArmConstants.PI_2 / InfeedPivotConstants.GEAR_RATIO;
        inputs.velRad = velocity.getValueAsDouble() * ArmConstants.PI_2 / InfeedPivotConstants.GEAR_RATIO;
        inputs.appliedV = motorVolts.getValueAsDouble();
        inputs.currentA = motorCurrent.getValueAsDouble();
        inputs.isConnected = motor.isConnected();
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
    public void resetPid(double posRad) {
        pid.reset(posRad);
    }

    @Override
    public void setPid(double posRad) {
        if (!Constants.CHAR_MODE)
            motor.setControl(voltControl.withOutput(pid.calculate(
                    motor.getPosition(true).getValueAsDouble() * ArmConstants.PI_2 / InfeedPivotConstants.GEAR_RATIO,
                    posRad)
                    + aFF.calculate(
                            motor.getPosition().getValueAsDouble() * ArmConstants.PI_2
                                    / InfeedPivotConstants.GEAR_RATIO,
                            motor.getVelocity(true).getValueAsDouble() * ArmConstants.PI_2
                                    / InfeedPivotConstants.GEAR_RATIO)));
    }

    @Override
    public void zeroPosition(double zeroPosRad) {
        motor.setPosition(zeroPosRad / ArmConstants.PI_2 * InfeedPivotConstants.GEAR_RATIO);
    }
}
