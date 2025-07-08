package frc.robot.subsystems.climber;

import frc.robot.util.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public double appliedVoltage = 0.0;
        public double currentAmps = 0.0;
        public MotorData motorData = MotorData.empty();
        public double motorPosition = 0.0;
        public boolean isConnected = false;
        public double position = 0.0;
        public double velocity = 0.0;
        public boolean connected = false;
    }


    default void updateInputs(ClimberIOInputs inputs) {
      
    }


    default void setVbus(double vBus) {

    }


    default void setVoltage(double volts) {

    }
    default void setPid(double position) {

    }

}