package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import java.util.Map;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.elevator.ElevatorStateTracker.*;
import frc.robot.util.SysIDUtil;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private ElevatorStateTracker stateTracker;
    private double targetVbus = 0.0, targetPostition = 0.0, targetVoltage = 0.0;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final Map<Boolean, Map<Direction, Command>> sysIDCommands;

    public static final record SimData(double currentAmps, double lengthMetres) {
    }

    public Elevator(ElevatorIO io) {
        this.io = io;
        sysIDCommands = SysIDUtil.generateTests(ElevatorConstants.sysIDConfig, this::runMotorsCommand, this);
    }

    public Command runMotorsCommand(double vbus) {
        return runOnce(() -> {
            targetVbus = vbus;
            stateTracker.setStateVBus(vbus);
        });
    }

    public Command runMotorsCommand(Voltage volts) {
        return runOnce(() -> {
            targetVoltage = volts.magnitude();
            stateTracker.setStateVoltage(volts.magnitude());
        });
    }

    public Command runToPositionCommand(double positionRot) {
        return runOnce(() -> {
            targetPostition = positionRot;
            stateTracker.state = ElevatorStates.PREPARE_TO_MOVE;
        });
    }

    public double getAccelaration() {
        return inputs.leaderAcceleration;
    }

    public Command sysIDTest(boolean dynamic, Direction direction) {
        return sysIDCommands.get(dynamic).get(direction);
    }

    public Command reefStateChangeCommand() {
        return runOnce(stateTracker::cycleReefState);
    }

    public Command reefCountChange(int shift) {
        return runOnce(() -> {
            stateTracker.shiftReefCount(shift);
        });
    }

    public Command runToReefPosition(boolean useReefCount) {
        if (useReefCount)
            stateTracker.applyReefCount();
        return runToPositionCommand(stateTracker.reefState.position);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        switch (stateTracker.state) {
            case IDLE:
                io.setVbus(0);
                break;
            case PREPARE_TO_MOVE:
                stateTracker.state = ElevatorStates.MOVING_POSITION;
                break;
            case MOVING_POSITION:
                io.setPid(targetPostition);
                break;
            case VBUS_BACKWARD:
            case VBUS_FORWARD:
                io.setVbus(targetVbus);
                break;
            case VOLTAGE_BACKWARD:
            case VOLTAGE_FORWARD:
                io.setVoltage(targetVoltage);
                break;
            case HOLDING_POSITION:
                break;
            default:
                break;
        }
    }

    public SimData getSimData() {
        return new SimData(inputs.leaderCurrentAmps + inputs.followerCurrentAmps,
                inputs.leaderPosition * ElevatorConstants.ROT_TO_METRES);
    }
}