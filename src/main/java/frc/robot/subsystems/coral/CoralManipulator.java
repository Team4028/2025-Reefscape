package frc.robot.subsystems.coral;

import java.util.Map;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.util.SysIDUtil;

public class CoralManipulator extends SubsystemBase {
    private final CoralManipulatorIO io;
    private final CoralManipulatorIOInputsAutoLogged inputs;
    private double targetVBus = 0.0, targetVoltage = 0.0;
    private final CoralManipulatorStateTracker stateTracker;
    private final Map<Boolean, Map<Direction, Command>> sysIDCommands;

    public CoralManipulator(CoralManipulatorIO io) {
        this.io = io;
        inputs = new CoralManipulatorIOInputsAutoLogged();
        stateTracker = new CoralManipulatorStateTracker();
        sysIDCommands = SysIDUtil.generateTests(CoralManipulatorConstants.sysIDConfig, this::runMotorVoltage, this);
    }

    public Command sysIDTest(boolean dynamic, Direction direction) {
        return sysIDCommands.get(dynamic).get(direction);
    }

    public Command runMotorCommand(double vbus) {
        return runOnce(() -> {
            targetVBus = vbus;
            stateTracker.setStateVBus(vbus);
        });
    }

    public void runMotorVoltage(double volts) {
        targetVoltage = volts;
        stateTracker.setStateVoltage(volts);
    }

    public Command runMotorVoltageCommand(double volts) {
        return runOnce(() -> runMotorVoltage(volts));
    }

    public BooleanSupplier hasGamePieceSupplier() {
        return () -> stateTracker.hasCoral;
    }

    @Override
    public void periodic() {
        stateTracker.state.execute(this);
        io.updateInputs(inputs);
        Logger.processInputs("Coral Manipulator", inputs);
    }

    @CreateState("off")
    public void stop() {
        io.setVbus(0);
    }

    @CreateState("vbus_forward")
    public void infeedVBus() {
        if (inputs.currentAmps < CoralManipulatorConstants.SUPPLY_LIMIT) {
            io.setVbus(targetVBus);
        } else {
            io.setVbus(0);
            stateTracker.hasCoral = true;
            stateTracker.state = CoralManipulatorStates.OFF;
        }
    }

    @CreateState("vbus_reverse")
    public void outfeedVBus() {
        io.setVbus(targetVBus);
        stateTracker.hasCoral = false;
    }

    @CreateState("voltage_forward")
    public void infeedVoltage() {
        if (inputs.currentAmps < CoralManipulatorConstants.SUPPLY_LIMIT) {
            io.setVoltage(targetVoltage);
        } else {
            io.setVbus(0);
            stateTracker.hasCoral = true;
            stateTracker.state = CoralManipulatorStates.OFF;
        }
    }

    @CreateState("voltage_reverse")
    public void outfeedVoltage() {
        io.setVoltage(targetVoltage);
        stateTracker.hasCoral = false;
    }

    public Trigger hasCoral() {
        return new Trigger(() -> stateTracker.hasCoral);
    }

    public void toggleHasCoral() {
        stateTracker.hasCoral = !stateTracker.hasCoral;
    }
}
