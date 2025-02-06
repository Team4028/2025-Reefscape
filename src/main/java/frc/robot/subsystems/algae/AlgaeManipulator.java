package frc.robot.subsystems.algae;

import java.lang.Thread.State;
import java.util.Map;

import com.bskd.annotations.CreateState;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.coral.CoralManipulatorConstants;
import frc.robot.subsystems.coral.CoralManipulatorIOInputsAutoLogged;
import frc.robot.subsystems.elevator.ElevatorStates;
import frc.robot.util.SysIDUtil;

public class AlgaeManipulator extends SubsystemBase {
    public enum AlgaeManipulatorStates {
        VBUS, VOLTAGE,
    }

    private double targetVoltage = 0.;
    private double targetVBus = 0;
    private final AlgaeManipulatorIO io;
    private final AlgaeManipulatorIOInputsAutoLogged inputs;
    private final Map<Boolean, Map<Direction, Command>> sysIDCommands;
    private AlgaeManipulatorStates state;

    public AlgaeManipulator(AlgaeManipulatorIO io) {
        this.io = io;
        inputs = new AlgaeManipulatorIOInputsAutoLogged();
        sysIDCommands = SysIDUtil.generateTests(AlgaeManipulatorConstants.sysIDConfig, this::runMotorCommand, this);
    }

    public Command runMotorVbusCommand(double vbus) {
        return runOnce(() -> {
            targetVBus = vbus;
            state = AlgaeManipulatorStates.VBUS;

        });
    }

    public Command runMotorCommand(Voltage volts) {
        return runOnce(() -> {
            targetVoltage = volts.magnitude();
            state = AlgaeManipulatorStates.VOLTAGE;
        });
    }

    @Override
    public void periodic() {
        switch (state) {
            case VBUS:
                io.setVbus(targetVBus);

                break;
            case VOLTAGE:
                io.setVoltage(targetVoltage);

                break;
            default:
                break;
        }
    }
}
