package frc.robot.subsystems.algae;

import java.util.Map;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.util.SysIDUtil;

public class AlgaeManipulator extends SubsystemBase {

    private double targetVoltage = 0.;
    private double targetVBus = 0;
    private final AlgaeManipulatorIO io;
    private final AlgaeManipulatorIOInputsAutoLogged inputs;
    private final Map<Boolean, Map<Direction, Command>> sysIDCommands;
    @AutoLogOutput
    private AlgaeManipulatorStates state = AlgaeManipulatorStates.OFF;
    @AutoLogOutput
    private boolean hasAlgae = false;

    public AlgaeManipulator(AlgaeManipulatorIO io) {
        this.io = io;
        inputs = new AlgaeManipulatorIOInputsAutoLogged();
        sysIDCommands = SysIDUtil.generateTests(AlgaeManipulatorConstants.sysIDConfig, this::runMotorVoltageCommand,
                this);
    }

    public Command sysIDCommand(boolean dynamic, Direction direction) {
        return sysIDCommands.get(dynamic).get(direction);
    }

    public Command runMotorCommand(double vbus) {
        return runOnce(() -> {
            targetVBus = vbus;
            state = vbus > 0 ? AlgaeManipulatorStates.VBUS_FORWARD
                    : (vbus < 0 ? AlgaeManipulatorStates.VBUS_REVERSE : AlgaeManipulatorStates.OFF);

        });
    }

    public Command runMotorVoltageCommand(double volts) {
        return runOnce(() -> {
            targetVoltage = volts;
            state = volts > 0 ? AlgaeManipulatorStates.VOLTAGE_FORWARD
                    : (volts < 0 ? AlgaeManipulatorStates.VOLTAGE_REVERSE : AlgaeManipulatorStates.OFF);
        });
    }

    public BooleanSupplier hasGamePieceSupplier() {
        return () -> hasAlgae;
    }

    @Override
    public void periodic() {
        state.execute(this);
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeManipulator", inputs);
    }

    @CreateState("vbus_forward")
    public void infeedVBus() {
        if (inputs.currentAmps < AlgaeManipulatorConstants.SUPPLY_LIMIT) {
            io.setVbus(targetVBus);
        } else {
            io.setVbus(0);
            hasAlgae = true;
            state = AlgaeManipulatorStates.OFF;
        }
    }

    @CreateState("vbus_reverse")
    public void outfeedVBus() {
        io.setVbus(targetVBus);
        hasAlgae = false;
    }

    @CreateState("voltage_forward")
    public void infeedVoltage() {
        if (inputs.currentAmps < AlgaeManipulatorConstants.SUPPLY_LIMIT) {
            io.setVoltage(targetVoltage);
        } else {
            io.setVbus(0);
            hasAlgae = true;
            state = AlgaeManipulatorStates.OFF;
        }
    }

    @CreateState("voltage_reverse")
    public void outfeedVoltage() {
        io.setVoltage(targetVoltage);
        hasAlgae = false;
    }

    @CreateState("off")
    public void stop() {
        io.setVbus(0);
    }
}
