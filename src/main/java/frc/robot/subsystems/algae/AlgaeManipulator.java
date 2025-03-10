package frc.robot.subsystems.algae;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeManipulator extends SubsystemBase {

    private double targetVBus = 0;
    private final AlgaeManipulatorIO io;
    private final AlgaeManipulatorIOInputsAutoLogged inputs;
    @AutoLogOutput
    private AlgaeManipulatorStates state = AlgaeManipulatorStates.OFF;
    @AutoLogOutput
    private boolean hasAlgae = false;

    private Timer currentLimitTimer = new Timer();

    public AlgaeManipulator(AlgaeManipulatorIO io) {
        this.io = io;
        inputs = new AlgaeManipulatorIOInputsAutoLogged();
    }

    public Command runMotorCommand(double vbus) {
        return runOnce(() -> {
            targetVBus = vbus;
            state = vbus > 0 ? AlgaeManipulatorStates.VBUS_FORWARD
                    : (vbus < 0 ? AlgaeManipulatorStates.VBUS_REVERSE : AlgaeManipulatorStates.OFF);

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
        if (inputs.currentAmps < AlgaeManipulatorConstants.SUPPLY_LIMIT
        || currentLimitTimer.get() <= AlgaeManipulatorConstants.CURRENT_LIMIT_DELAY_SEC) {
            currentLimitTimer.start();
            io.setVbus(targetVBus);
        } else {
            currentLimitTimer.stop();
            currentLimitTimer.reset();
            io.setVbus(0);
            hasAlgae = true;
            state = AlgaeManipulatorStates.OFF;
        }
    }

    @CreateState("vbus_reverse")
    public void outfeedVBus() {
        currentLimitTimer.stop();
        currentLimitTimer.reset();
        io.setVbus(targetVBus);
        hasAlgae = false;
    }

    @CreateState("off")
    public void stop() {
        if (hasAlgae) state = AlgaeManipulatorStates.HOLD;
        currentLimitTimer.stop();
        currentLimitTimer.reset();
        io.setVbus(0);
    }

    @CreateState("hold")
    public void hold() {
        currentLimitTimer.stop();
        currentLimitTimer.reset();
        io.setVoltage(0.5);
    }
}
