package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.MotorData;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double climberEncoderRaw = 0.0;
        public double climberEncoderRad = 0.0;
        public double climberVelocity = 0.0;
        public double appliedVoltage = 0.0;
        public double currentAmps = 0.0;
        public MotorData motorData = MotorData.empty();
    }

    


    public default void updateInputs(ClimberIOInputs inputs) {
      
    }


    public default void setVbus(double vBus) {

    }


    public default void setVoltage(double volts) {

    }
    public default void setPid(double position) {

    }
}