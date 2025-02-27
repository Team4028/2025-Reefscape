package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkMax;

import frc.robot.util.MotorData;

public class CoralManipulatorIOSparkMax implements CoralManipulatorIO {
    private final SparkMax motor;

    public CoralManipulatorIOSparkMax() {
        motor = new SparkMax(CoralManipulatorConstants.SparkMax.CAN_ID, CoralManipulatorConstants.SparkMax.MOTOR_TYPE);
        motor.configure(CoralManipulatorConstants.SparkMax.CONFIG, null, null);
    }

    @Override
    public void updateInputs(CoralManipulatorIOInputs inputs) {
        inputs.appliedVolts = motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.motorData = MotorData.getMotorData(motor);
    }

    @Override
    public void setVbus(double vBus) {
        motor.set(vBus);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
