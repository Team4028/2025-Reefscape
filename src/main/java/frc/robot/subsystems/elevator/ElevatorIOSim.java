package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.MathUtil;
import frc.robot.util.RobotSim;

public class ElevatorIOSim implements ElevatorIO {
    private final ProfiledPIDController pid;
    private final ElevatorFeedforward elevatorFF;

    private final ElevatorSim elevator;

    private double fakeAccel = 0, lastVel = 0, targetVolts = 0;

    public ElevatorIOSim() {
        pid = ElevatorConstants.simPidConstants.makeProfiledPIDController();

        elevatorFF = ElevatorConstants.simPidConstants.makeElevatorFeedforward();

        elevator = new ElevatorSim(
                LinearSystemId.createElevatorSystem(ElevatorConstants.simGearbox,
                        ElevatorConstants.CARRIAGE_MASS_Kg,
                        Units.inchesToMeters(ElevatorConstants.DRUM_RADIUS_IN), ElevatorConstants.MOTOR_TO_DRUM_RATIO),
                ElevatorConstants.simGearbox, 0, Units.inchesToMeters(ElevatorConstants.MAX_HEIGHT_INCHES), true, 0);
        RobotSim.registerCurrentInput("Elevator", elevator::getCurrentDrawAmps);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevator.setInput(targetVolts);
        elevator.update(0.02);
        fakeAccel = (elevator.getOutput(1) - lastVel) / 0.02;
        lastVel = elevator.getOutput(1);
        inputs.leaderPosition = Units.metersToInches(elevator.getOutput(0)) / ElevatorConstants.ROT_TO_IN;
        inputs.followerPosition = Units.metersToInches(elevator.getOutput(0)) / ElevatorConstants.ROT_TO_IN;
        inputs.leaderVelocity = Units.metersToInches(elevator.getOutput(1)) / ElevatorConstants.ROT_TO_IN;
        inputs.followerVelocity = Units.metersToInches(elevator.getOutput(1)) / ElevatorConstants.ROT_TO_IN;
        inputs.leaderAcceleration = Units.metersToInches(fakeAccel) / ElevatorConstants.ROT_TO_IN;
        inputs.followerAcceleration = Units.metersToInches(fakeAccel) / ElevatorConstants.ROT_TO_IN;
        inputs.elevatorPositionInches = inputs.leaderPosition * ElevatorConstants.ROT_TO_IN;
        inputs.elevatorVelocityInchesPerSecond = inputs.leaderVelocity * ElevatorConstants.ROT_TO_IN;
        inputs.leaderAppliedVolts = targetVolts;
        inputs.followerAppliedVolts = targetVolts;
        inputs.leaderCurrentAmps = elevator.getCurrentDrawAmps() / 2;
        inputs.followerCurrentAmps = elevator.getCurrentDrawAmps() / 2;
    }

    @Override
    public void setVoltage(double volts) {
        var rbv = RobotController.getBatteryVoltage();
        targetVolts = MathUtil.clamp(volts, -rbv, rbv);
    }

    @Override
    public void setVbus(double vBus) {
        setVoltage(vBus * RobotController.getBatteryVoltage());
    }

    @Override
    public void setPid(double positionInches) {
        setVoltage(
                pid.calculate(elevator.getOutput(0), Units.inchesToMeters(positionInches))
                        + elevatorFF.calculate(pid.getSetpoint().velocity));
    }
}
