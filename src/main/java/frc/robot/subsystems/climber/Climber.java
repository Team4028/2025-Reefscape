package frc.robot.subsystems.climber;

import com.bskd.annotations.CreateState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberEncoderIOCancoder encoderIO;
    private final ClimberStateTracker stateTracker;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final Timer runTimer = new Timer();
    private double targetVbus = 0.0, targetVoltage = 0.0;
    private double relativeControlInitialPos = 0.0;
    @AutoLogOutput
    private ClimberConstants.ClimberPositions targetPostition = ClimberConstants.ClimberPositions.ACQUIRE;

    public Climber(ClimberIO io, ClimberEncoderIOCancoder eio) {
        this.io = io;
        this.encoderIO = eio;
        encoderIO.updateInputs(inputs);
        stateTracker = new ClimberStateTracker();
        runTimer.stop();
        runTimer.reset();
    }

    public Command runVbusCommand(double vbus) {
        return runOnce(() -> {
            targetVbus = vbus;
            stateTracker.state = vbus > 0 ? ClimberStates.VBUS_FORWARD
                    : (vbus < 0 ? ClimberStates.VBUS_REVERSE : ClimberStates.OFF);

        });

        // return runOnce(() -> {
        //     targetVbus = vbus;
        //     stateTracker.setStateVBus(vbus);
        // });
    }

    public Command runVoltsCommand(double volts) {
        return runOnce(() -> {
            targetVoltage = volts;
            stateTracker.setStateVoltage(volts);
        });
    }

    public Command runPositionCommand(ClimberConstants.ClimberPositions pos) {
        return runOnce(() -> {
            targetPostition = pos;
            stateTracker.state = ClimberStates.POSITION;
            runTimer.start();
        });
    }

    public Command runPosRelative(double deltaRot) {
        return runOnce(() -> relativeControlInitialPos = inputs.motorPosition).andThen(runVbusCommand(0.5)).andThen(Commands.waitUntil(() -> inputs.motorPosition > relativeControlInitialPos + deltaRot)).andThen(runVbusCommand(0));
    }

    public Command deployCommand() {
        return runVbusCommand(0.5).andThen(Commands.waitUntil(() -> inputs.position > 0.95), runPosRelative(44));
    }

    public double getPosition() {
        return inputs.position;
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
        if (inputs.position - targetPostition.posRad < ClimberConstants.TOLERANCE/* || runTimer.get() > 2*/) {
            io.setVbus(0);
            runTimer.stop();
            runTimer.reset();
            stateTracker.state = ClimberStates.OFF;
        } else {
            if (targetPostition == ClimberConstants.ClimberPositions.CLIMB) {
                if (inputs.position > ClimberConstants.ClimberPositions.INTERMED.posRad) {
                    io.setVbus(0.7);
                } else {
                    io.setVbus(0.2 + 0.5 * ((ClimberConstants.ClimberPositions.CLIMB.posRad - inputs.position) / (ClimberConstants.ClimberPositions.CLIMB.posRad - ClimberConstants.ClimberPositions.INTERMED.posRad)));
                }
            } else {
                io.setVbus(0.1);
            }
        }
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
        encoderIO.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

}
