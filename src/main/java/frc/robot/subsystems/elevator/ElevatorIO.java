package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.GetMotorData.MotorData;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double leaderPosition = 0.0;
        public double leaderVelocity = 0.0;
        public double leaderAcceleration = 0.0;
        public double leaderAppliedVolts = 0.0;
        public double leaderCurrentAmps = 0.0;
        public double followerPosition = 0.0;
        public double followerVelocity = 0.0;
        public double followerAcceleration = 0.0;
        public double followerAppliedVolts = 0.0;
        public double followerCurrentAmps = 0.0;
        public double elevatorPositionInches = 0.0;
        public double elevatorVelocityInchesPerSecond = 0.0;
        public double velocityRadPerSec = 0.0;
        public MotorData leaderData = MotorData.empty();
        public MotorData followerData = MotorData.empty();
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void setPid(double positionInches) {
    }

    public default void setVoltage(double volts) {
    }

    public default void setVbus(double vBus) {
    }
    public default void runOpenLoop(double output) {
        
    }
}
