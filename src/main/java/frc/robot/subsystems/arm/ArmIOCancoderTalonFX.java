package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.generated.TunerConstants;
import frc.robot.util.MotorData;

//...

// This will start Redux CANLink manually for Java

public class ArmIOCancoderTalonFX implements ArmIO {

    private final TalonFX motor = new TalonFX(ArmConstants.TalonFX.MOTOR_ID);
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> motorCurrent = motor.getSupplyCurrent();
    private final StatusSignal<Angle> motorPosition = motor.getPosition();
    private final StatusSignal<AngularVelocity> motorVel = motor.getVelocity();

    private final CANcoder cancoder = new CANcoder(ArmConstants.Cancoder.CAN_ID,
            TunerConstants.DrivetrainConstants.CANBusName);
    private final StatusSignal<Angle> encoderPosition = cancoder.getAbsolutePosition();

    private final VoltageOut voltRequest = new VoltageOut(0).withEnableFOC(ArmConstants.USE_FOC);
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(ArmConstants.USE_FOC);
    private final MotionMagicVoltage pidControl = new MotionMagicVoltage(0).withSlot(0);

    public ArmIOCancoderTalonFX() {
        motor.getConfigurator().apply(ArmConstants.TalonFX.motorConfigs, 0.25);
        motor.getConfigurator().apply(ArmConstants.TalonFX.pidConfigs, 0.25);
        motor.getConfigurator().apply(ArmConstants.TalonFX.mmConfigs, 0.25);
        cancoder.getConfigurator().apply(ArmConstants.Cancoder.config);
        new Thread(() -> {
            while (RobotController.getCANStatus().percentBusUtilization < 0.9) {
                System.out.println("Waiting to zero arm");
            }
            if (DriverStation.isDisabled()) {
                setPosition(-0.87);
                DriverStation.reportWarning("ZEROED MOTOR " + motor.getDeviceID(), false);
            }
        }).start();

        BaseStatusSignal.setUpdateFrequencyForAll(100, motorVolts, motorCurrent, motorPosition, motorVel,
                encoderPosition);
        motor.optimizeBusUtilization();
        cancoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorVel, motorPosition, motorVolts, motorCurrent, encoderPosition);
        inputs.appliedVoltage = motorVolts.getValueAsDouble();
        inputs.currentAmps = motorCurrent.getValueAsDouble();
        inputs.armMotorVelocityRotPerSec = motorVel.getValueAsDouble();
        inputs.armVelocityRotPerSec = motorVel.getValueAsDouble() / ArmConstants.GEAR_RATIO;
        inputs.armAngleRad = getArmAngleRad();
        inputs.armEncoderRad = getEncoderPositionRad();
        inputs.armEncoderRaw = getRawEncoderPositon();
        inputs.motorData = MotorData.getMotorData(motor);
        inputs.isConnected = motor.isConnected();
        ArmIO.super.updateInputs(inputs);
    }

    public void setBrake(boolean isBrake) {
        motor.getConfigurator()
                .apply(ArmConstants.TalonFX.motorConfigs
                        .withNeutralMode(isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast));
    }

    public void initEncoder() {
        BaseStatusSignal.refreshAll(encoderPosition);
        motor.setPosition(encoderPosition.getValueAsDouble() * ArmConstants.GEAR_RATIO);
    }

    public void setPosition(double positionRad) {
        motor.setPosition((positionRad / ArmConstants.PI_2) * ArmConstants.GEAR_RATIO);
    }

    public double getRawEncoderPositon() {
        return encoderPosition.getValueAsDouble();
    }

    public double getEncoderPositionRad() {
        return motorPosition.getValueAsDouble() / ArmConstants.GEAR_RATIO * ArmConstants.PI_2;
    }

    public double getArmAngleRad() {
        return getEncoderPositionRad() - 0.2;
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