package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagFaults;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.MotorData;

//...

// This will start Redux CANLink manually for Java

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
    private final MotionMagicVoltage pidControl = new MotionMagicVoltage(0).withSlot(0);

    public ArmIOCanEncoderTalonFX() {
        BaseStatusSignal.setUpdateFrequencyForAll(10, motorVolts, motorCurrent, motorVel);
        motor.optimizeBusUtilization();
        settings.setVelocityFilterWidth(25);
        settings.setInvertDirection(false);
        settings.setDisableZeroButton(false);
        settings.setVelocityFilterWidth(25);
        settings.setPositionFramePeriod(0.020);
        canMag.setSettings(settings);

        canMag.setPartyMode(10);
        motor.getConfigurator().apply(ArmConstants.TalonFX.motorConfigs);
        motor.getConfigurator().apply(ArmConstants.TalonFX.pidConfigs);
        motor.getConfigurator().apply(ArmConstants.TalonFX.mmConfigs);
        CanandEventLoop.getInstance();
        new Thread(() -> {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException ignored) {
            }

            initEncoder();
            System.out.println(String.format("Successfully initialized TalonFX %d Position", motor.getDeviceID()));
        }).start();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorVel, motorVolts, motorCurrent);
        inputs.appliedVoltage = motorVolts.getValueAsDouble();
        inputs.currentAmps = motorCurrent.getValueAsDouble();
        inputs.armMotorVelocityRotPerSec = motorVel.getValueAsDouble();
        inputs.armVelocityRotPerSec = motorVel.getValueAsDouble() / ArmConstants.GEAR_RATIO;
        inputs.armAngleRad = getArmAngleRad();
        inputs.armEncoderRad = getEncoderPositionRad();
        inputs.armEncoderRaw = getRawEncoderPositon();
        inputs.motorData = MotorData.getMotorData(motor);
        inputs.canMagPosition = canMag.getPosition();
        inputs.canMagVelocity = canMag.getVelocity();
        inputs.canMagInRange = canMag.magnetInRange();
        ArmIO.super.updateInputs(inputs);
    }

    public void initEncoder() {
        motor.setPosition(canMag.getAbsPosition() * ArmConstants.GEAR_RATIO);
    }

    public double getRawEncoderPositon() {
        return motor.getPosition(true).getValueAsDouble() / ArmConstants.GEAR_RATIO;
    }

    public double getEncoderPositionRad() {
        var rot = motor.getPosition(true).getValueAsDouble() / ArmConstants.GEAR_RATIO;
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

    @Override
    public void setPID(double position) {
        motor.setControl(pidControl.withPosition((position / ArmConstants.PI_2) * ArmConstants.GEAR_RATIO));
    }
}