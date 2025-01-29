package frc.robot.subsystems.coral;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.coral.CoralManipulatorStateTracker.CoralStates;
import frc.robot.util.SysIDUtil;

public class CoralManipulator extends SubsystemBase {
    private final CoralManipulatorIO io;
    private final CoralManipulatorIOInputsAutoLogged inputs;
    private double targetVBus = 0.0, targetVoltage = 0.0;
    private final CoralManipulatorStateTracker stateTracker;
    private final Map<Boolean, Map<Direction, Command>> sysIDCommands;

    public static final record SimData(double currentAmps) {
    }

    public CoralManipulator(CoralManipulatorIO io) {
        this.io = io;
        inputs = new CoralManipulatorIOInputsAutoLogged();
        stateTracker = new CoralManipulatorStateTracker();
        sysIDCommands = SysIDUtil.generateTests(CoralManipulatorConstants.sysIDConfig, this::runMotorCommand, this);
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

    public Command runMotorCommand(Voltage volts) {
        return runOnce(() -> {
            targetVoltage = volts.magnitude();
            stateTracker.setStateVoltage(volts.magnitude());
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral Manipulator", inputs);
        switch (stateTracker.state) {
            case OFF:
                io.setVbus(0);
                break;
            case VBUS_FORWARD:
                if (inputs.currentAmps < CoralManipulatorConstants.SUPPLY_LIMIT) {
                    io.setVbus(targetVBus);
                } else {
                    io.setVbus(0);
                    stateTracker.hasCoral = true;
                    stateTracker.state = CoralStates.OFF;
                }
                break;
            case VBUS_REVERSE:
                io.setVbus(targetVBus);
                stateTracker.hasCoral = false;
                break;
            case VOLTAGE_FORWARD:
                if (inputs.currentAmps < CoralManipulatorConstants.SUPPLY_LIMIT) {
                    io.setVoltage(targetVoltage);
                } else {
                    io.setVbus(0);
                    stateTracker.hasCoral = true;
                    stateTracker.state = CoralStates.OFF;
                }
                break;
            case VOLTAGE_REVERSE:
                io.setVoltage(targetVoltage);
                stateTracker.hasCoral = false;
                break;
            default:
                break;
        }
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    public SimData getSimData() {
        return new SimData(inputs.currentAmps);
    }
}
