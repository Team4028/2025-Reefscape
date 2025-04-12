package frc.robot.subsystems.groundinfeed;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grond extends SubsystemBase {
    private final GrondIO io;
    private final GrondTOFIO tokio;
    private final GrondIOInputsAutoLogged inputs;
    private final GrondTOFIOInputsAutoLogged tofInputs;
    private double targetVbus = 0.0;
    private GrondStates state = GrondStates.OFF;
    private boolean hasCoral = false;
    private Timer currentLimitTimer = new Timer();

    public Grond(GrondIO io, GrondTOFIO tofIo) {
        this.io = io;
        this.tokio = tofIo;
        inputs = new GrondIOInputsAutoLogged();
        tofInputs = new GrondTOFIOInputsAutoLogged();
    }

    @AutoLogOutput
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
        if (tofInputs.range > GrondConstants.PWFTimeOfFlight.TOF_RANGE_THRESH/*inputs.currentAmps < GrondConstants.STATOR_LIMIT || currentLimitTimer.get() <= GrondConstants.CURRENT_LIMIT_DELAY_SEC*/) {
            if (inputs.currentAmps >= GrondConstants.STATOR_LIMIT) {
                currentLimitTimer.start();
            } else {
                currentLimitTimer.stop();
                currentLimitTimer.reset();
            }
            io.setVbus(targetVbus);
        } else {
            currentLimitTimer.stop();
            currentLimitTimer.reset();
            hasCoral = true;
            state = GrondStates.HOLD;
        }
    }

    public void setHasCoral(boolean hasCoral) {
        this.hasCoral = hasCoral;
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
        tokio.updateInputs(tofInputs);
        Logger.processInputs("Ground Infeed/Motor", inputs);
        Logger.processInputs("Ground Infeed/TOF Sensor", tofInputs);
    }
}
