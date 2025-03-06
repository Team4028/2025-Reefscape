package frc.robot.subsystems.elevator;

import java.util.Map;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Armistice.ArmisticePositions;
import frc.robot.util.MathUtils;
import frc.robot.util.SysIDUtil;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private ElevatorStateTracker stateTracker;
    private double targetVbus = 0.0, targetVoltage = 0.0;
    @AutoLogOutput
    private double targetPostitionInches = ArmisticePositions.STOW.getElevatorPositionInches(0);
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final Map<Boolean, Map<Direction, Command>> sysIDCommands;

    public Elevator(ElevatorIO io) {
        this.io = io;
        stateTracker = new ElevatorStateTracker();
        sysIDCommands = SysIDUtil.generateTests(ElevatorConstants.sysIDConfig, this::runMotorsVoltage, this);
    }

    public void runMotors(double vbus) {
        targetVbus = vbus;
        stateTracker.setStateVBus(vbus);
    }

    public BooleanSupplier atTargetPosition() {
        return () -> Math.abs(targetPostitionInches
                - inputs.leaderPosition * ElevatorConstants.ROT_TO_IN) <= ElevatorConstants.PID_TOLERANCE;
    }

    public void runMotorsVoltage(double volts) {
        targetVoltage = volts;
        stateTracker.setStateVoltage(volts);
    }

    public void runToPosition(double positionInches) {
        targetPostitionInches = positionInches;
        stateTracker.state = stateTracker.state == ElevatorStates.MOVING_POSITION
                || stateTracker.state == ElevatorStates.PREPARE_TO_MOVE ? ElevatorStates.MOVING_POSITION
                        : ElevatorStates.PREPARE_TO_MOVE;
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
        return targetPostitionInches;
    }

    public double getCurrentPosition() {
        return inputs.leaderPosition * ElevatorConstants.ROT_TO_IN;
    }

    public Command reefCountChange(int shift) {
        return runOnce(() -> {
            stateTracker.shiftReefCount(shift);
        });
    }

    public void runToReefPosition(boolean useReefCount) {
        if (useReefCount)
            stateTracker.applyReefCount();
        runToPosition(stateTracker.reefState.position);
    }

    public void nudge(double amount) {
        runToPosition(targetPostitionInches + amount);
    }

    @Override
    public void periodic() {
        stateTracker.state.execute(this);
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Elevator/ElevatorPositionInchesFrieldly", MathUtils.roundToPlace(inputs.elevatorPositionInches, 3));
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
        io.setPid(targetPostitionInches);
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
        return Units.inchesToMeters(inputs.elevatorPositionInches);
    }
}