package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagFaults;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.GetMotorData;

public class ArmIOCanEncoderTalonFX implements ArmIO {
    private final Canandmag canMag = new Canandmag(5);
    private CanandmagSettings settings = new CanandmagSettings();
    CanandmagFaults faults;

    private final TalonFX motor = new TalonFX(ArmConstants.TalonFX.MOTOR_ID);
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> motorCurrent = motor.getSupplyCurrent();
    private final StatusSignal<AngularVelocity> motorVel = motor.getVelocity();

    private final VoltageOut voltRequest = new VoltageOut(0).withEnableFOC(ArmConstants.USE_FOC);
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(ArmConstants.USE_FOC);

    public ArmIOCanEncoderTalonFX() {
        settings.setVelocityFilterWidth(25);
        settings.setInvertDirection(false);
        settings.setDisableZeroButton(false);
        settings.setVelocityFilterWidth(25);
        settings.setPositionFramePeriod(0.020);

        canMag.setPartyMode(10);
        motor.getConfigurator().apply(ArmConstants.TalonFX.motorConfigs);
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
        inputs.motorData = GetMotorData.getTalonFXData(motor);
        inputs.canMagPosition = canMag.getPosition();
        inputs.canMagVelocity = canMag.getVelocity();
        inputs.canMagInRange = canMag.magnetInRange();
        ArmIO.super.updateInputs(inputs);
    }

    public double getRawEncoderPositon() {
        return canMag.getPosition();
    }

    public double getEncoderPositionRad() {
        var rot = canMag.getPosition() - 0;
        rot = rot > 0 ? rot : 1 + rot;
        return rot * ArmConstants.PI_2;
    }

    public double getArmAngleRad() {
        var rad = getEncoderPositionRad() - ArmConstants.PI_1_2;
        rad = rad > 0 ? rad : ArmConstants.PI_2 + rad;
        return rad;
    }

    public void setPosition(double newPosition) {
        canMag.setPosition(newPosition);
    }

    public void zeroCanMag() {
        canMag.zeroAll();
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