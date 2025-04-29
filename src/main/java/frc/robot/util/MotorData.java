package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import frc.robot.subsystems.arm.ArmConstants;
import lombok.With;

@With
public record MotorData(double positionRad, double velocityRadPerSec, double tempCelcius) {

    public static MotorData empty() {
        return new MotorData(0, 0, 0);
    }

    @Override
    public String toString() {
        return "Position: " + positionRad + ", Velocity: " + velocityRadPerSec + ", Temperature: " + tempCelcius;
    }

    public static MotorData getMotorData(TalonFX talon) {
        return new MotorData(talon.getPosition(true).getValueAsDouble() * ArmConstants.PI_2,
                talon.getVelocity(true).getValueAsDouble() * ArmConstants.PI_2,
                talon.getDeviceTemp(true).getValueAsDouble());
    }

    public static MotorData getMotorData(SparkMax spark, AbsoluteEncoder sparkEncoder) {
        return new MotorData(sparkEncoder.getPosition() * ArmConstants.PI_2,
                sparkEncoder.getVelocity() * ArmConstants.PI_2, spark.getMotorTemperature());
    }

    public static MotorData getMotorData(SparkMax spark) {
        return new MotorData(0, 0, spark.getMotorTemperature());
    }
}
