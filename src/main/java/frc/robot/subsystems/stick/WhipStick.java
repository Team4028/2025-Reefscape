package frc.robot.subsystems.stick;

import com.bskd.annotations.CreateState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MiscUtils;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

@ExtensionMethod(MiscUtils.class)
public class WhipStick extends SubsystemBase {
    private final WhipStickIO io;
    private final WhipStickIOInputsAutoLogged inputs;
    private double targetVBus = 0.0;
    private final WhipStickStateTracker stateTracker;
    private boolean isGettingAlgae = false;
    private final Timer currentLimitTimer = new Timer();
    private final Timer coralHoldTimer = new Timer();
    private final double coralHoldTimePreset = 10; //seconds
    private final double coralHoldCurrentAmps = 15;

    public WhipStick(WhipStickIO io) {
        this.io = io;
        inputs = new WhipStickIOInputsAutoLogged();
        stateTracker = new WhipStickStateTracker();
    }

    public Command runMotorCommandAlgae(double vbus) {
        return runOnce(() -> {
            if (stateTracker.state == WhipStickStates.HOLD)
                return;
            targetVBus = vbus;
            isGettingAlgae = true;
            stateTracker.setStateVBus(vbus);
        });
    }

    public Command runMotorCommand(double vbus) {
        return runOnce(() -> {
            if (stateTracker.state == WhipStickStates.HOLD && vbus == 0)
                return;
            targetVBus = vbus;
            isGettingAlgae = false;
            stateTracker.setStateVBus(vbus);
        });
    }

    public Command stopMotorCommand() {
        return runOnce (() -> {
            isGettingAlgae = false;
            stateTracker.state = WhipStickStates.OFF;
        });
    }
    @AutoLogOutput
    public BooleanSupplier hasAlgae() {
        return hasGamePieceSupplier().bsand(() -> isGettingAlgae);
    }

    @AutoLogOutput
    public BooleanSupplier hasGamePieceSupplier() {
        return () -> stateTracker.hasGP;
    }

    public void setHasGamepiece(boolean has) {
        stateTracker.hasGP = has;
    }

    @Override
    public void periodic() {
        stateTracker.state.execute(this);
        io.updateInputs(inputs);
        Logger.processInputs("Coral Manipulator", inputs);
    }

    @CreateState("off")
    public void stop() {
        if (isGettingAlgae) {
            stateTracker.state = WhipStickStates.HOLD;
        } else if (stateTracker.hasGP) {
            io.setVbus(0);
        } else {
            io.setVbus(0);
        }
        currentLimitTimer.stop();
        currentLimitTimer.reset();
        coralHoldTimer.stop();
        coralHoldTimer.reset();
    }

    @CreateState("hold")
    public void hold() {
        if (coralHoldTimer.get() >= coralHoldTimePreset && !isGettingAlgae) {
            stateTracker.state = WhipStickStates.OFF;
            targetVBus = 0;
        }
        currentLimitTimer.stop();
        currentLimitTimer.reset();
        if (!isGettingAlgae) {
            if (io instanceof WhipStickIOTalonFX fx)
                fx.setCurrent(coralHoldCurrentAmps);
            else
                io.setVbus(0.1);
        } else {
            if (io instanceof WhipStickIOTalonFX iofx) {
                iofx.setCurrent(50);
            } else {
                io.setVbus(0.95);
            }
        }
    }

    @CreateState("vbus_forward")
    public void infeedVBus() {
        if ((isGettingAlgae
                && (inputs.currentAmps < WhipStickConstants.STATOR_LIMIT_ALGAE
                || currentLimitTimer.get() <= WhipStickConstants.CURRENT_LIMIT_DELAY_SEC))
                || (!isGettingAlgae && (inputs.currentAmps < WhipStickConstants.STATOR_LIMIT_CORAL)
                || currentLimitTimer.get() <= WhipStickConstants.CURRENT_LIMIT_DELAY_SEC)) {
            currentLimitTimer.start();
            io.setVbus(targetVBus);
        } else {
            currentLimitTimer.stop();
            currentLimitTimer.reset();
            stateTracker.hasGP = true;
            stateTracker.state = WhipStickStates.HOLD;
            if (stateTracker.hasGP && !isGettingAlgae) {
                coralHoldTimer.reset();
                coralHoldTimer.start();
            }
        }
    }

    @CreateState("vbus_reverse")
    public void outfeedVBus() {
        currentLimitTimer.stop();
        currentLimitTimer.reset();
        coralHoldTimer.stop();
        coralHoldTimer.reset();
        io.setVbus(targetVBus);
        stateTracker.hasGP = false;
    }
}
