package frc.robot.subsystems.cage;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.MotorData;

public interface CageIO {
    @AutoLog
    class CageIOInputs {
        public double appliedVoltage = 0.0;
        public double currentAmps = 0.0;
        public MotorData motorData = MotorData.empty();
        public boolean isConnected = false;
    }

    

    default void updateInputs(CageIOInputs inputs) {
      
    }


    default void setVbus(double vBus) {

    }


    default void setVoltage(double volts) {

    }
    default void setPid(double position) {

    }
}