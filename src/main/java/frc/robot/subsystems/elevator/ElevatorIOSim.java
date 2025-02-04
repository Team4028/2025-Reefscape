package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ElevatorIOSim implements ElevatorIO {
    private final ProfiledPIDController pid;
    private final ElevatorFeedforward elevatorFF;

    private final ElevatorSim elevator;

    private double fakeAccel = 0, lastVel = 0, targetVolts = 0;

    public ElevatorIOSim() {
        pid = ElevatorConstants.pidConstants.makeProfiledPIDController();

        elevatorFF = ElevatorConstants.pidConstants.makeElevatorFeedforward();

        elevator = new ElevatorSim(
                LinearSystemId.createElevatorSystem(ElevatorConstants.simGearbox, ElevatorConstants.CARRIAGE_MASS_KG,
                        ElevatorConstants.DRUM_RADIUS, ElevatorConstants.MOTOR_TO_DRUM_RATIO),
                ElevatorConstants.simGearbox, 0, ElevatorConstants.MAX_HEIGHT_METERS, true, 0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevator.setInput(targetVolts);
        elevator.update(0.02);
        fakeAccel = (elevator.getOutput(1) - lastVel) / 0.02;
        lastVel = elevator.getOutput(1);
        inputs.leaderPosition = elevator.getOutput(0) / ElevatorConstants.ROT_TO_METRES;
        inputs.followerPosition = elevator.getOutput(0) / ElevatorConstants.ROT_TO_METRES;
        inputs.leaderVelocity = elevator.getOutput(1) / ElevatorConstants.ROT_TO_METRES;
        inputs.followerVelocity = elevator.getOutput(1) / ElevatorConstants.ROT_TO_METRES;
        inputs.leaderAcceleration = fakeAccel / ElevatorConstants.ROT_TO_METRES;
        inputs.followerAcceleration = fakeAccel / ElevatorConstants.ROT_TO_METRES;
        inputs.leaderAppliedVolts = targetVolts;
        inputs.followerAppliedVolts = targetVolts;
        inputs.leaderCurrentAmps = elevator.getCurrentDrawAmps() / 2;
        inputs.followerCurrentAmps = elevator.getCurrentDrawAmps() / 2;
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevator.getCurrentDrawAmps()));
    }

    @Override
    public void setVoltage(double volts) {
        targetVolts = volts;
    }

    @Override
    public void setVbus(double vBus) {
        setVoltage(vBus * RobotController.getBatteryVoltage());
    }

    @Override
    public void setPid(double positionRot) {
        setVoltage(
                pid.calculate(elevator.getOutput(0) / ElevatorConstants.ROT_TO_METRES, positionRot) + elevatorFF.calculate(pid.getSetpoint().velocity));
    }
}
