package frc.robot.subsystems.algae;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.RobotController;

public class AlgaeManipulatorIOTalonSRX implements AlgaeManipulatorIO {
    private TalonSRX motor;
    public AlgaeManipulatorIOTalonSRX() {
        motor = new TalonSRX(AlgaeManipulatorConstants.TalonSRX.CAN_ID);
        motor.setInverted(AlgaeManipulatorConstants.TalonSRX.INVERT);
    }
    @Override
    public void updateInputs(AlgaeManipulatorIOInputs inputs) {
        inputs.appliedVolts = motor.getMotorOutputVoltage();
        inputs.currentAmps = motor.getSupplyCurrent();
    }
    @Override
    public void setVbus(double vbus) {
        motor.set(TalonSRXControlMode.PercentOutput, vbus);
    }
    @Override
    public void setVoltage(double volts) {
        setVbus(volts / RobotController.getBatteryVoltage());
    }
}
