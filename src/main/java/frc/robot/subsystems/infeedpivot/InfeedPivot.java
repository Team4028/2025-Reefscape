package frc.robot.subsystems.infeedpivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.infeedpivot.InfeedPivotConstants.InfeedPivotPositions;

import static frc.robot.subsystems.infeedpivot.InfeedPivotConstants.InfeedPivotPositions.*;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

public class InfeedPivot extends SubsystemBase {
    private final InfeedPivotMotorIO motorIO;
    private final InfeedPivotEncoderIO encoderIO;
    private final InfeedPivotIOMotorInputsAutoLogged motorInputs;
    private final InfeedPivotEncoderIOInputsAutoLogged encoderInputs;
    private double targetVbus = 0;
    private double targetPositionRad = UP.posRad;

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

        new Thread(() -> {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException ignored) {}
            encoderIO.updateInputs(encoderInputs);
            motorIO.zeroPosition(encoderInputs.positionRad);
        }).start();
    }

    public Command runUp() {
        return runToPositionCommand(InfeedPivotPositions.UP.posRad).alongWith(Commands.runOnce(() -> up = true));
    }

    public Command runDown() {
        return runMotorCommand(-0.2).alongWith(Commands.runOnce(() -> up = false));
    }

    public Command hhhTest() {
        return runUp().andThen(Commands.waitSeconds(0.001)).andThen(runDown()).andThen(runDown()).andThen(Commands.waitSeconds(0.001)).repeatedly();
    }

    public Command waitUntilInTolerance(double toleranceRad) {
        return Commands.waitUntil(() -> Math.abs(motorInputs.positionRad - InfeedPivotPositions.HANDOFF.posRad) <= toleranceRad);
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
        return runOnce(() -> {
            state = InfeedPivotStates.ZEROING;
        });
    }

    public Command runToPositionCommand(double positionRad) {
        return runOnce(() -> {
            targetPositionRad = positionRad;
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

    @CreateState("vbus_forward")
    @CreateState("vbus_reverse")
    @CreateState("off")
    @CreateState("holding_down")
    public void runTargetVbus() {
        if (state == InfeedPivotStates.HOLDING_DOWN) {
            motorIO.setVBus(-0.5);
            return;
        } else if (motorInputs.positionRad < 0.7) {
            if (motorInputs.positionRad < 0.05) {
                state = InfeedPivotStates.HOLDING_DOWN;
            }
            motorIO.setVBus(0);
            return;
        }
        if ((motorInputs.positionRad / ArmConstants.PI_2 * InfeedPivotConstants.GEAR_RATIO)
                - 1 < InfeedPivotConstants.TalonFX.softLimits.ReverseSoftLimitThreshold) {
            targetVbus = 0;
            state = InfeedPivotStates.OFF;
        }
        motorIO.setVBus(targetVbus);
    }

    @CreateState("position")
    public void runTargetPosition() {
        motorIO.setPid(targetPositionRad);
    }

    @Override
    public void periodic() {
        state.execute(this);
        motorIO.updateInputs(motorInputs);
        encoderIO.updateInputs(encoderInputs);
        Logger.processInputs("Infeed Pivot/Motor", motorInputs);
        Logger.processInputs("Infeed Pivot/Encoder", encoderInputs);
    }
}