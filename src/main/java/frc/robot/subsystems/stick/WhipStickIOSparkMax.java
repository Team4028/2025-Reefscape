package frc.robot.subsystems.stick;

import com.revrobotics.spark.SparkMax;

import frc.robot.util.MotorData;

public class WhipStickIOSparkMax implements WhipStickIO {
    private final SparkMax motor;

    public WhipStickIOSparkMax() {
        motor = new SparkMax(WhipStickConstants.SparkMax.CAN_ID, WhipStickConstants.SparkMax.MOTOR_TYPE);
        motor.configure(WhipStickConstants.SparkMax.CONFIG, null, null);
    }

    @Override
    public void updateInputs(WhipStickIOInputs inputs) {
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
