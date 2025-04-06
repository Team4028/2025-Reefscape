package frc.robot.subsystems.groundinfeed;

import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grond extends SubsystemBase {
    private final GrondIO io;
    private final GrondIOInputsAutoLogged inputs;
    private double targetVbus = 0.0;
    private GrondStates state = GrondStates.OFF;

    public Grond(GrondIO io) {
        this.io = io;
        inputs = new GrondIOInputsAutoLogged();
    }

    public Command runMotorCommand(double vbus) {
        return runOnce(() -> {
            targetVbus = vbus;
            state = vbus > 0 ? GrondStates.VBUS_FORWARD : vbus < 0 ? GrondStates.VBUS_REVERSE : GrondStates.OFF;
        });
    }

    @CreateState("vbus_forward")
    @CreateState("vbus_reverse")
    @CreateState("off")
    public void runVbus() {
        io.setVbus(targetVbus);
    }

    @Override
    public void periodic() {
        state.execute(this);
        io.updateInputs(inputs);
        Logger.processInputs("Ground Infeed", inputs);
    }
}
