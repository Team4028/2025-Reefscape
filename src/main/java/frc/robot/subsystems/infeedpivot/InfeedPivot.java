package frc.robot.subsystems.infeedpivot;

import com.bskd.annotations.CreateState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.infeedpivot.InfeedPivotConstants.InfeedPivotPositions;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

import static frc.robot.subsystems.infeedpivot.InfeedPivotConstants.InfeedPivotPositions.UP;

public class InfeedPivot extends SubsystemBase {
    private final InfeedPivotMotorIO motorIO;
    private final InfeedPivotEncoderIO encoderIO;
    private final InfeedPivotIOMotorInputsAutoLogged motorInputs;
    private final InfeedPivotEncoderIOInputsAutoLogged encoderInputs;
    private double targetVbus = 0;
    private double targetPositionRad = UP.posRad;
    private double targetVoltage = 0;
    @AutoLogOutput
    private boolean up = true;

    @AutoLogOutput
    private InfeedPivotStates state = InfeedPivotStates.POSITION;

    public InfeedPivot(InfeedPivotMotorIO motorIO, InfeedPivotEncoderIO encoderIO) {
        this.motorIO = motorIO;
        this.encoderIO = encoderIO;
        motorInputs = new InfeedPivotIOMotorInputsAutoLogged();
        encoderInputs = new InfeedPivotEncoderIOInputsAutoLogged();
        encoderIO.updateInputs(encoderInputs);
        motorIO.resetPid(motorInputs.positionRad);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException ignored) {
            }
            encoderIO.updateInputs(encoderInputs);
            motorIO.zeroPosition(encoderInputs.positionRad);
        }).start();
    }

    public Command runUp() {
        return runOnce(() -> motorIO.setPIDConstants(InfeedPivotConstants.pidConfig)).andThen(runToPositionCommand(InfeedPivotPositions.UP.posRad).alongWith(Commands.runOnce(() -> up = true)));
    }

    public Command runUpWithCoral() {
        return runOnce(() -> up = true).andThen(runVoltageCommand(10));
    }

    public Command runUpClimb() {
        return runToPositionCommand(InfeedPivotPositions.CLIMB.posRad).alongWith(Commands.runOnce(() -> up = true));
    }

    public Command runDown() {
        return runMotorCommand(-0.2).alongWith(Commands.runOnce(() -> up = false));
    }

    public Command hhhTest() {
        return runUp().andThen(Commands.waitSeconds(0.001)).andThen(runDown()).andThen(runDown()).andThen(Commands.waitSeconds(0.001)).repeatedly();
    }

    public Command runVoltageCommand(double voltage) {
        return runOnce(() -> {
            targetVoltage = voltage;
            state = voltage > 0 ? InfeedPivotStates.VOLTAGE_FORWARD : voltage < 0 ? InfeedPivotStates.VOLTAGE_REVERSE : InfeedPivotStates.OFF;
        });
    }

    public Command waitUntilInTolerance(double toleranceRad) {
        return Commands.waitUntil(() -> Math.abs(motorInputs.positionRad - InfeedPivotPositions.HANDOFF.posRad) <= toleranceRad);
    }

    public BooleanSupplier isDownPositional() {
        return () -> state == InfeedPivotStates.HOLDING_DOWN;
    }

    public BooleanSupplier isUp() {
        return () -> up;
    }

    public Command runMotorCommand(double vbus) {
        return runOnce(() -> {
            targetVbus = vbus;
            state = vbus > 0 ? InfeedPivotStates.VBUS_FORWARD
                    : (vbus < 0 ? InfeedPivotStates.VBUS_REVERSE : InfeedPivotStates.OFF);
        });
    }

    public Command zeroMotorEncoder() {
        return runOnce(() -> state = InfeedPivotStates.ZEROING);
    }

    public double getPosition() {
        return encoderInputs.positionRad;
    }

    public Command runToPositionCommand(double positionRad) {
        return runOnce(() -> {
            targetPositionRad = positionRad;
            motorIO.resetPid(motorInputs.positionRad);
            state = InfeedPivotStates.POSITION;
        });
    }

    @CreateState("zeroing")
    public void zero() {
        motorIO.setVBus(0);
        targetVbus = 0;
        motorIO.zeroPosition(encoderInputs.positionRad);
        state = up ? InfeedPivotStates.POSITION : InfeedPivotStates.OFF;
    }

    @CreateState("voltage_forward")
    @CreateState("voltage_reverse")
    public void runTargetVoltage() {
        motorIO.setVoltage(targetVoltage);
    }

    @CreateState("vbus_forward")
    @CreateState("vbus_reverse")
    @CreateState("off")
    @CreateState("holding_down")
    public void runTargetVbus() {
        if (state == InfeedPivotStates.HOLDING_DOWN) {
            motorIO.setVBus(-0.9);
            return;
        } else if (motorInputs.positionRad < 1.1) {
            if (motorInputs.positionRad < 0.1) {
                state = InfeedPivotStates.HOLDING_DOWN;
            }
            motorIO.setVBus(0);
            return;
        }
        motorIO.setVBus(targetVbus);
    }

    @CreateState("position")
    public void runTargetPosition() {
        motorIO.setPid(targetPositionRad);
    }

    @Override
    public void periodic() {
        if (!Constants.CHAR_MODE)
            state.execute(this);
        motorIO.updateInputs(motorInputs);
        encoderIO.updateInputs(encoderInputs);
        Logger.processInputs("Infeed Pivot/Motor", motorInputs);
        Logger.processInputs("Infeed Pivot/Encoder", encoderInputs);
    }
}