package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.MotorData;

public class ArmIODIOEncoderTalonFX implements ArmIO {
    private final DutyCycleEncoder encoder;
    private double lastPosition = 0.0;
    private final TalonFX motor = new TalonFX(ArmConstants.TalonFX.MOTOR_ID);
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> motorAmps = motor.getSupplyCurrent();
    private final StatusSignal<AngularVelocity> motorVel = motor.getVelocity();

    public ArmIODIOEncoderTalonFX() {
        DigitalInput di = new DigitalInput(ArmConstants.DIOEncoder.DIO_PIN);
        encoder = new DutyCycleEncoder(di);
        encoder.setInverted(ArmConstants.DIOEncoder.INVERTED);
        motor.getConfigurator().apply(ArmConstants.TalonFX.motorConfigs);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorVolts, motorAmps, motorVel);
        inputs.appliedVoltage = motorVolts.getValueAsDouble();
        inputs.currentAmps = motorAmps.getValueAsDouble();
        inputs.armMotorVelocityRotPerSec = motorVel.getValueAsDouble();
        inputs.armAngleRad = getArmAngleRad();
        inputs.armEncoderRad = getEncoderPositionRad();
        inputs.armEncoderRaw = getRawEncoderPositon();
        double fakeVel = (encoder.get() - lastPosition) / 0.02;
        lastPosition = encoder.get();
        inputs.armVelocityRotPerSec = fakeVel;
        inputs.motorData = MotorData.getMotorData(motor);
        inputs.isConnected = motor.isConnected();
        ArmIO.super.updateInputs(inputs);
    }

    public double getRawEncoderPositon() {
        return encoder.get();
    }

    public double getEncoderPositionRad() {
        var rot = getRawEncoderPositon() - 0;
        rot = rot > 0 ? rot : 1 + rot;
        return rot * ArmConstants.PI_2;
    }

    public double getArmAngleRad() {
        var rad = getEncoderPositionRad() - ArmConstants.PI_1_2;
        rad = rad > 0 ? rad : ArmConstants.PI_2 + rad;
        return rad;
    }
}
