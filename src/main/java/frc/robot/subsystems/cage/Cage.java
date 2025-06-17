package frc.robot.subsystems.cage;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MiscUtils;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod(MiscUtils.class)
public class Cage extends SubsystemBase {
    private final CageIO io;
    private final CageStateTracker stateTracker;
    private double targetVbus = 0.0, targetVoltage = 0.0;

    @AutoLogOutput
    private double targetPostitionInches = 0.0;
    private final CageIOInputsAutoLogged inputs = new CageIOInputsAutoLogged();

    public Cage(CageIO io) {
        this.io = io;
        stateTracker = new CageStateTracker();
        io.updateInputs(inputs);
    }

    public Command runVbusCommand(double vbus) {
        return runOnce(() -> {
            targetVbus = vbus;
            stateTracker.state = vbus > 0 ? CageStates.VBUS_FORWARD
                    : (vbus < 0 ? CageStates.VBUS_REVERSE : CageStates.OFF);

        });

        // return runOnce(() -> {
        // targetVbus = vbus;
        // stateTracker.setStateVBus(vbus);
        // });
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
            stateTracker.state = CageStates.POSITION;
        });
    }

    @CreateState("vbus_forward")
    @CreateState("vbus_reverse")
    public void runTargetVbus() {
        // if (inputs.currentAmps < CageConstants.MOTOR_CURRENT_LIMIT) {
        // io.setVbus(targetVbus);
        // } else {
        // stateTracker.state = CageStates.OFF;
        // }
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

    @Override
    public void periodic() {
        stateTracker.state.execute(this);
        io.updateInputs(inputs);
        Logger.processInputs("Cage", inputs);
    }

}
