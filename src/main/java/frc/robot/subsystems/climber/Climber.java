package frc.robot.subsystems.climber;

import java.util.Map;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberStateTracker stateTracker;
    private double targetVbus = 0.0, targetVoltage = 0.0;

    @AutoLogOutput
    private double targetPostitionInches = 0.0;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;
        stateTracker = new ClimberStateTracker();
        io.updateInputs(inputs);
    }

    public Command runVbusCommand(double vbus) {
        return runOnce(() -> {
            targetVbus = vbus;
            stateTracker.setStateVBus(vbus);
        });
    }

    public Command runVoltsCommand(double volts) {
        return runOnce(() -> {
            targetVoltage = volts;
            stateTracker.setStateVoltage(volts);
        });
    }

    public Command runPositionCommand(double position) {
        return runOnce(() -> {
            targetPostitionInches = position;
            stateTracker.state = ClimberStates.POSITION;
        });
    }

    @CreateState("vbus_forward")
    @CreateState("vbus_reverse")
    public void runTargetVbus() {
        io.setVbus(targetVbus);
    }

    @CreateState("off")
    public void stop() {
        io.setVbus(0);
    }

    @CreateState("position")
    public void runTargetPosition() {
        io.setPid(targetPostitionInches);
    }

    @CreateState("voltage_reverse")
    @CreateState("voltage_forward")
    public void runTargetVolts() {
        io.setVoltage(targetVoltage);
    }

}
