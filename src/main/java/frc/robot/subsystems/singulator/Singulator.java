package frc.robot.subsystems.singulator;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Singulator extends SubsystemBase {
    private final SingulatorIO io;
    private final SingulatorIOInputsAutoLogged inputs;
    private double targetVbus = 0.0;
    private DigitalInput limSwitch = new DigitalInput(SingulatorConstants.LSWITCH_ID);
    private SingulatorStates state = SingulatorStates.OFF;

    public Singulator(SingulatorIO io) {
        this.io = io;
        inputs = new SingulatorIOInputsAutoLogged();
    }

    public Command runMotorCommand(double vbus) {
        return runOnce(() -> {
            targetVbus = vbus;
            state = vbus > 0 ? SingulatorStates.VBUS_FORWARD : vbus < 0 ? SingulatorStates.VBUS_REVERSE : SingulatorStates.OFF;
        });
    }

    @AutoLogOutput
    public BooleanSupplier hasGamepieceSupplier() {
        return limSwitch::get;
    }

    public boolean getLSwitch() {
        return limSwitch.get();
    }

    @CreateState("vbus_forward")
    public void infeedVbus() {
        if (limSwitch.get()) {
            state = SingulatorStates.HOLD;
        } else io.setVBus(targetVbus);
    }

    @CreateState("off")
    public void stop() {
        io.setVBus(0);
        if (hasGamepieceSupplier().getAsBoolean()) {
            state = SingulatorStates.HOLD;
        }
    }

    @CreateState("hold")
    public void hold() {
        io.setVBus(0.1);
        if (!hasGamepieceSupplier().getAsBoolean()) {
            state = SingulatorStates.OFF;
        }
    }

    @CreateState("vbus_reverse")
    public void outfeedVBus() {
        io.setVBus(targetVbus);
    }

    @Override
    public void periodic() {
        state.execute(this);
        io.updateInputs(inputs);
        Logger.processInputs("Singulator", inputs);
    }
}
