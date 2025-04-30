package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagFaults;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.MathUtils;
import frc.robot.util.MotorData;

//...

// This will start Redux CANLink manually for Java

public class ArmIOCanEncoderTalonFX implements ArmIO {
    private final Canandmag canMag = new Canandmag(5);
    CanandmagFaults faults;

    private final TalonFX motor = new TalonFX(ArmConstants.TalonFX.MOTOR_ID);
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> motorCurrent = motor.getSupplyCurrent();
    private final StatusSignal<Angle> motorPosition = motor.getPosition();
    private final StatusSignal<AngularVelocity> motorVel = motor.getVelocity();

    private final VoltageOut voltRequest = new VoltageOut(0).withEnableFOC(ArmConstants.USE_FOC);
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(ArmConstants.USE_FOC);
    private final MotionMagicVoltage pidControl = new MotionMagicVoltage(0).withSlot(0);

    public ArmIOCanEncoderTalonFX() {
        CanandmagSettings settings = new CanandmagSettings();
        settings.setVelocityFilterWidth(25);
        settings.setInvertDirection(false);
        settings.setDisableZeroButton(false);
        settings.setVelocityFilterWidth(25);
        settings.setPositionFramePeriod(0.020);
        canMag.setSettings(settings);
        canMag.setPartyMode(10);
        motor.getConfigurator().apply(ArmConstants.TalonFX.motorConfigs, 0.25);
        motor.getConfigurator().apply(ArmConstants.TalonFX.pidConfigs, 0.25);
        motor.getConfigurator().apply(ArmConstants.TalonFX.mmConfigs, 0.25);
        CanandEventLoop.getInstance();
        new Thread(() -> {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException ignored) {
            }

            while (motor.getVelocity(true).getValueAsDouble() != 0);
            initEncoder();
            System.out.printf("Successfully initialized TalonFX %d Position%n", motor.getDeviceID());
        }).start();

        BaseStatusSignal.setUpdateFrequencyForAll(100, motorVolts, motorCurrent, motorPosition, motorVel);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorVel, motorPosition, motorVolts, motorCurrent);
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
        inputs.isConnected = motor.isConnected();
        ArmIO.super.updateInputs(inputs);
    }

    public void setBrake(boolean isBrake) {
        motor.getConfigurator()
                .apply(ArmConstants.TalonFX.motorConfigs.withNeutralMode(NeutralModeValue.valueOf(isBrake ? 1 : 0)));
    }

    public void initEncoder() {
        motor.setPosition(MathUtils.cyclicRange(canMag.getAbsPosition() - 0.6366, -0.5, 1, 1) * ArmConstants.GEAR_RATIO);
    }

    public double getRawEncoderPositon() {
        motorPosition.refresh();
        return motorPosition.getValueAsDouble() / ArmConstants.GEAR_RATIO;
    }

    public double getEncoderPositionRad() {
        motorPosition.refresh();
        // rot = rot > 0 ? rot : 1 + rot;
        // return rot * ArmConstants.PI_2;
        return motorPosition.getValueAsDouble() / ArmConstants.GEAR_RATIO * ArmConstants.PI_2;
    }

    public double getArmAngleRad() {
        return getEncoderPositionRad() - 0.2;
        // rad = rad > 0 ? rad : ArmConstants.PI_2 + rad;
        // return rad;
    }

    @Override
    public void setVBus(double vBus) {
        motor.setControl(dutyCycleOut.withOutput(vBus));
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltRequest.withOutput(volts));
    }

    public void setArmAccel(double accel) {
        motor.getConfigurator().apply(ArmConstants.TalonFX.mmConfigs.withMotionMagicAcceleration(accel));
    }

    @Override
    public void setPID(double position) {
        motor.setControl(pidControl.withPosition((position / ArmConstants.PI_2) * ArmConstants.GEAR_RATIO));
    }
}