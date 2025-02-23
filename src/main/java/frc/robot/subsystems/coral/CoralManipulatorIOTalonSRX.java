package frc.robot.subsystems.coral;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.RobotController;

public class CoralManipulatorIOTalonSRX implements CoralManipulatorIO {
    private final TalonSRX motor;

    public CoralManipulatorIOTalonSRX() {
        motor = new TalonSRX(CoralManipulatorConstants.TalonSRX.CAN_ID);
        motor.setInverted(CoralManipulatorConstants.TalonSRX.INVERT);
        motor.setNeutralMode(CoralManipulatorConstants.TalonSRX.NEUTRALMODE);
    }

    @Override
    public void updateInputs(CoralManipulatorIOInputs inputs) {
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
