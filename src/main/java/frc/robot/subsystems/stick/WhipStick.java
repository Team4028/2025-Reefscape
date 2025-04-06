package frc.robot.subsystems.stick;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WhipStick extends SubsystemBase {
    private final WhipStickIO io;
    private final WhipStickIOInputsAutoLogged inputs;
    private double targetVBus = 0.0;
    private final WhipStickStateTracker stateTracker;
    private boolean isGettingAlgae = false;
    private Timer currentLimitTimer = new Timer();

    public WhipStick(WhipStickIO io) {
        this.io = io;
        inputs = new WhipStickIOInputsAutoLogged();
        stateTracker = new WhipStickStateTracker();
    }

    public Command runMotorUnlimited(double vbus) {
        return runOnce(() -> {
            targetVBus = vbus;
            isGettingAlgae = true;
            stateTracker.setStateVBus(vbus);
        });
    }

    public Command runMotorCommand(double vbus) {
        return runOnce(() -> {
            targetVBus = vbus;
            isGettingAlgae = false;
            stateTracker.setStateVBus(vbus);
        });
    }

    @AutoLogOutput
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
        if (stateTracker.hasCoral)
            stateTracker.state = WhipStickStates.HOLD;
        io.setVbus(0);
        currentLimitTimer.stop();
        currentLimitTimer.reset();
    }

    @CreateState("hold")
    public void hold() {
        currentLimitTimer.stop();
        currentLimitTimer.reset();
        if (io instanceof WhipStickIOTalonFX iofx) {
            iofx.setCurrent(50);
        } else {
            io.setVbus(0.95);
        }
    }

    @CreateState("vbus_forward")
    public void infeedVBus() {
        if ((isGettingAlgae
                && (inputs.currentAmps < 40 || currentLimitTimer.get() <= WhipStickConstants.CURRENT_LIMIT_DELAY_SEC))
                || (!isGettingAlgae && (inputs.currentAmps < WhipStickConstants.STATOR_LIMIT
                        || currentLimitTimer.get() <= WhipStickConstants.CURRENT_LIMIT_DELAY_SEC))) {
            currentLimitTimer.start();
            io.setVbus(targetVBus);
        } else {
            currentLimitTimer.stop();
            currentLimitTimer.reset();
            io.setVbus(0);
            stateTracker.hasCoral = true;
            stateTracker.state = isGettingAlgae ? WhipStickStates.HOLD : WhipStickStates.OFF;
        }
    }

    @CreateState("vbus_reverse")
    public void outfeedVBus() {
        currentLimitTimer.stop();
        currentLimitTimer.reset();
        io.setVbus(targetVBus);
        stateTracker.hasCoral = false;
    }
}
