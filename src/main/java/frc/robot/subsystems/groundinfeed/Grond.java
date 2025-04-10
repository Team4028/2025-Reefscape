package frc.robot.subsystems.groundinfeed;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grond extends SubsystemBase {
    private final GrondIO io;
    private final GrondIOInputsAutoLogged inputs;
    private double targetVbus = 0.0;
    private GrondStates state = GrondStates.OFF;
    private boolean hasCoral = false;
    private Timer currentLimitTimer = new Timer();

    public Grond(GrondIO io) {
        this.io = io;
        inputs = new GrondIOInputsAutoLogged();
    }

    public BooleanSupplier hasGamepieceSupplier() {
        return () -> hasCoral;
    }

    public Command runMotorCommand(double vbus) {
        return runOnce(() -> {
            targetVbus = vbus;
            state = vbus > 0 ? GrondStates.VBUS_FORWARD : vbus < 0 ? GrondStates.VBUS_REVERSE : GrondStates.OFF;
        });
    }

    @CreateState("vbus_forward")
    public void infeedVbus() {
        if (inputs.currentAmps < GrondConstants.STATOR_LIMIT || currentLimitTimer.get() <= GrondConstants.CURRENT_LIMIT_DELAY_SEC) {
            currentLimitTimer.start();
            io.setVbus(targetVbus);
        } else {
            currentLimitTimer.stop();
            currentLimitTimer.reset();
            hasCoral = true;
            state = GrondStates.HOLD;
        }
    }

    @CreateState("off")
    public void stop() {
        if (hasCoral)
            state = GrondStates.HOLD;
        io.setVbus(0);
        currentLimitTimer.stop();
        currentLimitTimer.reset();
    }

    @CreateState("hold")
    public void hold() {
        currentLimitTimer.stop();
        currentLimitTimer.reset();
        io.setCurrent(15);
    }

    @CreateState("vbus_reverse")
    public void outfeedVBus() {
        currentLimitTimer.stop();
        currentLimitTimer.reset();
        io.setVbus(targetVbus);
        hasCoral = false;
    }


    @Override
    public void periodic() {
        state.execute(this);
        io.updateInputs(inputs);
        Logger.processInputs("Ground Infeed", inputs);
    }
}
