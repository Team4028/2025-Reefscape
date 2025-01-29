package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ArmIOSparkEncoderTalonFX implements ArmIO {
    private final SparkMax encoderReader;
    private final AbsoluteEncoder encoder;
    private final TalonFX motor = new TalonFX(ArmConstants.MOTOR_ID);
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> motorCurrent = motor.getSupplyCurrent();
    private final StatusSignal<AngularVelocity> motorVel = motor.getVelocity();

    private final VoltageOut voltRequest = new VoltageOut(0).withEnableFOC(ArmConstants.USE_FOC);
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(ArmConstants.USE_FOC);

    public ArmIOSparkEncoderTalonFX() {
        encoderReader = new SparkMax(ArmConstants.ENCODER_ID, MotorType.kBrushless);
        encoderReader.configure(ArmConstants.encoderConfig, null, null);
        encoder = encoderReader.getAbsoluteEncoder();
        motor.getConfigurator().apply(ArmConstants.motorConfigs);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorVel, motorVolts, motorCurrent);
        inputs.appliedVoltage = motorVolts.getValueAsDouble();
        inputs.currentAmps = motorCurrent.getValueAsDouble();
        inputs.armMotorVelocityRotPerSec = motorVel.getValueAsDouble();
        inputs.armAngleRad = getArmAngleRad();
        inputs.armEncoderRad = getEncoderPositionRad();
        inputs.armEncoderRaw = getRawEncoderPositon();
        inputs.armVelocityRotPerSec = encoder.getVelocity();
    }

    public double getRawEncoderPositon() {
        return encoder.getPosition();
    }

    public double getEncoderPositionRad() {
        var rot = encoder.getPosition() - 0.48972 + 0.25;
        rot = rot > 0 ? rot : 1 + rot;
        return rot * ArmConstants.PI_2;
    }

    public double getArmAngleRad() {
        var rad = getEncoderPositionRad() - ArmConstants.PI_1_2;
        rad = rad > 0 ? rad : ArmConstants.PI_2 + rad;
        return rad;
    }

    @Override
    public void setVBus(double vBus) {
        motor.setControl(dutyCycleOut.withOutput(vBus));
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltRequest.withOutput(volts));
    }
}