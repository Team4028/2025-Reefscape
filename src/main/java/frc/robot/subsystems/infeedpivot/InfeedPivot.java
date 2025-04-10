package frc.robot.subsystems.infeedpivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.infeedpivot.InfeedPivotConstants.InfeedPivotPositions.*;

import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

public class InfeedPivot extends SubsystemBase {
    private final InfeedPivotMotorIO motorIO;
    private final InfeedPivotEncoderIO encoderIO;
    private final InfeedPivotIOMotorInputsAutoLogged motorInputs;
    private final InfeedPivotEncoderIOInputsAutoLogged encoderInputs;
    private double targetVbus = 0;
    private double targetPositionRad = UP.posRad;
    private InfeedPivotStates state = InfeedPivotStates.OFF;

    public InfeedPivot(InfeedPivotMotorIO motorIO, InfeedPivotEncoderIO encoderIO) {
        this.motorIO = motorIO;
        this.encoderIO = encoderIO;
        motorInputs = new InfeedPivotIOMotorInputsAutoLogged();
        encoderInputs = new InfeedPivotEncoderIOInputsAutoLogged();
        encoderIO.updateInputs(encoderInputs);
        motorIO.zeroPosition(encoderInputs.positionRad);
    }

    public Command runMotorCommand(double vbus) {
        return runOnce(() -> {
            targetVbus = vbus;
            state = vbus > 0 ? InfeedPivotStates.VBUS_FORWARD : (vbus < 0 ? InfeedPivotStates.VBUS_REVERSE : InfeedPivotStates.OFF);
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
        state = InfeedPivotStates.OFF;
    }

    @CreateState("vbus_forward")
    @CreateState("vbus_reverse")
    @CreateState("off")
    public void runTargetVbus() {
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