package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.util.SysIDUtil;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private ElevatorStateTracker stateTracker;
    private double targetVbus = 0.0, targetVoltage = 0.0;
    @AutoLogOutput 
    private double targetPostition = 0.0;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final Map<Boolean, Map<Direction, Command>> sysIDCommands;

    public Elevator(ElevatorIO io) {
        this.io = io;
        stateTracker = new ElevatorStateTracker();
        sysIDCommands = SysIDUtil.generateTests(ElevatorConstants.sysIDConfig, this::runMotorsCommand, this);
    }

    public Command runMotorsCommand(double vbus) {
        return runOnce(() -> {
            targetVbus = vbus;
            stateTracker.setStateVBus(vbus);
        });
    }

    public BooleanSupplier atTargetPosition() {
        return () -> Math.abs(targetPostition - inputs.leaderPosition) <= ElevatorConstants.PID_TOLERANCE;
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

    public double getTargetPosition() {
        return inputs.leaderPosition;
    }

    public double getCurrentPosition() {
        return inputs.leaderPosition;
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

    public Command nudgeCommand(double amount) {
        return runOnce(() -> {
            targetPostition += amount;
            stateTracker.state = ElevatorStates.PREPARE_TO_MOVE;
        });
    }

    @Override
    public void periodic() {
        stateTracker.state.execute(this);
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    @CreateState("off")
    public void stop() {
        io.setVbus(0);
    }

    @CreateState("holding_position")
    public void hold() {
    }

    @CreateState("prepare_to_move")
    public void movementPreparation() {
        stateTracker.state = ElevatorStates.MOVING_POSITION;
    }

    @CreateState("moving_position")
    public void runTargetPosition() {
        io.setPid(targetPostition);
    }

    @CreateState("vbus_forward")
    @CreateState("vbus_reverse")
    public void runTargetVBus() {
        io.setVbus(targetVbus);
    }

    @CreateState("voltage_forward")
    @CreateState("voltage_reverse")
    public void runTargetVoltage() {
        io.setVoltage(targetVoltage);
    }

    public double getSimPos() {
        return inputs.leaderPosition * ElevatorConstants.ROT_TO_METRES;
    }
}