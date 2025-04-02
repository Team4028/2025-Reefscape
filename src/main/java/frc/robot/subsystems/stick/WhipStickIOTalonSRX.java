package frc.robot.subsystems.stick;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.RobotController;

public class WhipStickIOTalonSRX implements WhipStickIO {
    private final TalonSRX motor;

    public WhipStickIOTalonSRX() {
        motor = new TalonSRX(WhipStickConstants.TalonSRX.CAN_ID);
        motor.setInverted(WhipStickConstants.TalonSRX.INVERT);
        motor.setNeutralMode(WhipStickConstants.TalonSRX.NEUTRALMODE);
    }

    @Override
    public void updateInputs(WhipStickIOInputs inputs) {
        inputs.appliedVolts = motor.getMotorOutputVoltage();
        inputs.currentAmps = motor.getSupplyCurrent();
    }

    @Override
    public void setVbus(double vBus) {
        motor.set(TalonSRXControlMode.PercentOutput, vBus);
    }

    @Override
    public void setVoltage(double volts) {
        setVbus(volts / RobotController.getBatteryVoltage());
    }
}
