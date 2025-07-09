package frc.robot.subsystems.groundinfeed;

import com.bskd.annotations.CreateState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MathUtils;
import frc.robot.util.MiscUtils;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

@ExtensionMethod(MiscUtils.class)
public class Grond extends SubsystemBase {
    private final GrondIO ioleft, ioright;
    private final GrondTOFIO tof1io;
    private final GrondIOInputsAutoLogged inputsLeft;
    private final GrondIOInputsAutoLogged inputsRight;
    private final GrondTOFIOInputsAutoLogged tof1Inputs;
    private final Timer currentLimitTimer = new Timer();
    private double targetVbusLeft = 0.0;
    private double targetVbusRight = 0.0;
    @Getter
    @AutoLogOutput
    private GrondStates state = GrondStates.OFF;
    @Setter
    private boolean hasCoral = false;
    private final BooleanSupplier pivotDown; // bad

    public Grond(GrondIO ioleft, GrondIO ioright, GrondTOFIO tof1Io, BooleanSupplier pivotDown) {
        this.ioleft = ioleft;
        this.ioright = ioright;
        this.tof1io = tof1Io;
        this.pivotDown = pivotDown;
        inputsLeft = new GrondIOInputsAutoLogged();
        inputsRight = new GrondIOInputsAutoLogged();
        tof1Inputs = new GrondTOFIOInputsAutoLogged();
    }

    @AutoLogOutput
    public BooleanSupplier hasGamepieceSupplier() {
        return () -> hasCoral;
    }

    @AutoLogOutput
    public BooleanSupplier hasGamepieceSupplierRawTOF() {
        return () -> tof1Inputs.range <= GrondConstants.PWFTimeOfFlight.TOF_RANGE_THRESH;
    }

    public Command directRunMotorCommand(double vbus) {
        return runOnce(() -> {
            targetVbusRight = vbus;
            targetVbusLeft = vbus * GrondConstants.RIGHT_TO_LEFT_RATIO;
            ioright.setVbus(targetVbusRight);
            ioleft.setVbus(targetVbusLeft);
            state = vbus > 0 ? GrondStates.VBUS_FORWARD : vbus < 0 ? GrondStates.VBUS_REVERSE : GrondStates.OFF;
        });
    }

    public Command runMotorCommand(double vbus) {
        return runOnce(() -> {
            targetVbusRight = vbus;
            targetVbusLeft = vbus * GrondConstants.RIGHT_TO_LEFT_RATIO;
            state = vbus > 0 ? GrondStates.VBUS_FORWARD : vbus < 0 ? GrondStates.VBUS_REVERSE : GrondStates.OFF;
        });
    }

    public Command unjamCommand() {
        return runOnce(() -> state = GrondStates.UNJAM).alongWith(Commands.waitSeconds(0.05))
                .until(hasGamepieceSupplierRawTOF()).finallyDo(() -> state = GrondStates.VBUS_FORWARD);
    }

    public BooleanSupplier isJammed() {
        return currLimitHasGP().bsand(hasGamepieceSupplier().bsnot());
    }

    public BooleanSupplier currLimitHasGP() {
        return () -> MathUtils.average(inputsLeft.currentAmps, inputsRight.currentAmps)
                - GrondConstants.TalonFX.JAM_STATOR >= -5;
    }

    @CreateState("vbus_forward")
    public void infeedVbus() {
        if ((tof1Inputs.range >= GrondConstants.PWFTimeOfFlight.TOF_RANGE_THRESH) || !pivotDown.getAsBoolean()) {
            if (MathUtils.average(inputsLeft.currentAmps, inputsRight.currentAmps) >= GrondConstants.STATOR_LIMIT) {
                currentLimitTimer.start();
            } else {
                currentLimitTimer.stop();
                currentLimitTimer.reset();
            }
            ioleft.setVbus(targetVbusLeft);
            ioright.setVbus(targetVbusRight);
        } else {
            currentLimitTimer.stop();
            currentLimitTimer.reset();
            hasCoral = true;
            state = GrondStates.HOLD;
        }
    }

    @CreateState("unjam")
    public void unjam() {
        ioleft.setVbus(-0.6);
        ioright.setVbus(0.9);
        currentLimitTimer.stop();
        currentLimitTimer.reset();
    }

    @CreateState("off")
    public void stop() {
        if (hasCoral)
            state = GrondStates.HOLD;
        ioleft.setVbus(0);
        ioright.setVbus(0);
        currentLimitTimer.stop();
        currentLimitTimer.reset();
    }

    @CreateState("hold")
    public void hold() {
        currentLimitTimer.stop();
        currentLimitTimer.reset();
        ioleft.setCurrent(15);
        ioright.setCurrent(15);
        if (!hasCoral)
            state = GrondStates.OFF;
    }

    public void setBrake(boolean isBrake) {
        if (ioleft instanceof GrondIOTalonFX fx) {
            fx.setBrakeMode(isBrake);
        }
        if (ioright instanceof GrondIOTalonFX fx) {
            fx.setBrakeMode(isBrake);
        }
    }

    @CreateState("vbus_reverse")
    public void outfeedVBus() {
        currentLimitTimer.stop();
        currentLimitTimer.reset();
        ioleft.setVbus(targetVbusLeft);
        ioright.setVbus(targetVbusLeft);
        hasCoral = false;
    }

    @Override
    public void periodic() {
        state.execute(this);
        ioleft.updateInputs(inputsLeft);
        ioright.updateInputs(inputsRight);
        tof1io.updateInputs(tof1Inputs);
        Logger.processInputs("Ground Infeed/MotorLeft", inputsLeft);
        Logger.processInputs("Ground Infeed/MotorRight", inputsRight);
        Logger.processInputs("Ground Infeed/TOF Sensor", tof1Inputs);
    }
}
