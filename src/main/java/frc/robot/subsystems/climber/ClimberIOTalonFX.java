package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.util.GetMotorData.MotorData;

public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX motor;
    private final StatusSignal<Voltage> motorVolts;
    private final StatusSignal<Current> motorAmps;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Angle> position;
    private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(ClimberConstants.USE_FOC);
    private final DutyCycleOut vbusControl = new DutyCycleOut(0).withEnableFOC(ClimberConstants.USE_FOC);
    private final PositionVoltage pidControl = new PositionVoltage(0).withEnableFOC(ClimberConstants.USE_FOC)
            .withSlot(0);

    public ClimberIOTalonFX() {
        motor = new TalonFX(ClimberConstants.TalonFX.MOTOR_ID);
        motorVolts = motor.getMotorVoltage();
        motorAmps = motor.getSupplyCurrent();
        velocity = motor.getVelocity();
        position = motor.getPosition();
    }

   @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberEncoderRaw = position.getValueAsDouble();
        inputs.climberEncoderRad = getEncoderPositonRad();
        inputs.climberVelocity = velocity.getValueAsDouble();
        inputs.appliedVoltage = motorVolts.getValueAsDouble();
        inputs.currentAmps = motorAmps.getValueAsDouble();
        inputs.motorData = MotorData.empty();
    }



    public double getEncoderPositonRad() {
        var rad = position.getValueAsDouble() * ((Math.PI * 2) / 4096); 
        return rad;
    }

    public double getClimberPositionRad() {
        var rad = getEncoderPositonRad() - ClimberConstants.CLIMBER_OFFSET;
        return rad;
    }


    @Override
    public void setVbus(double vBus) {
        motor.setControl(vbusControl.withOutput(vBus));
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setPid(double position) {
        motor.setControl(pidControl.withPosition(position));
    }

}
