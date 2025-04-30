package frc.robot.subsystems.groundinfeed;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.MathUtils;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MiscUtils;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod(MiscUtils.class)
public class Grond extends SubsystemBase {
    private final GrondIO ioleft, ioright;
    private final GrondTOFIO tokio;
    private final GrondIOInputsAutoLogged inputsLeft;
    private final GrondIOInputsAutoLogged inputsRight;
    private final GrondTOFIOInputsAutoLogged tofInputs;
    private double targetVbusLeft = 0.0;
    private double targetVbusRight = 0.0;
    @AutoLogOutput
    private GrondStates state = GrondStates.OFF;
    @Setter
    private boolean hasCoral = false;
    private final Timer currentLimitTimer = new Timer();
    private final Timer tofDebounceTimer = new Timer();
    private final BooleanSupplier pivotDown;

    public Grond(GrondIO ioleft, GrondIO ioright, GrondTOFIO tofIo, BooleanSupplier pivotDown) {
        this.pivotDown = pivotDown;
        this.ioleft = ioleft;
        this.ioright = ioright;
        this.tokio = tofIo;
        inputsLeft = new GrondIOInputsAutoLogged();
        inputsRight = new GrondIOInputsAutoLogged();
        tofInputs = new GrondTOFIOInputsAutoLogged();
    }

    @AutoLogOutput
    public BooleanSupplier hasGamepieceSupplier() {
        return () -> hasCoral;
    }

    @AutoLogOutput
    public BooleanSupplier hasGamepieceSupplierRawTOF() {
        return () -> tofInputs.range <= GrondConstants.PWFTimeOfFlight.TOF_RANGE_THRESH;
    }

    public Command runMotorCommand(double vbus) {
        return runOnce(() -> {
            targetVbusRight = vbus;
            targetVbusLeft = vbus * GrondConstants.RIGHT_TO_LEFT_RATIO;
            state = vbus > 0 ? GrondStates.VBUS_FORWARD : vbus < 0 ? GrondStates.VBUS_REVERSE : GrondStates.OFF;
        });
    }

    public Command unjamCommand() {
//        return runOnce(() -> state = GrondStates.UNJAM).alongWith(Commands.waitSeconds(0.05)).finallyDo(() -> state = GrondStates.VBUS_FORWARD);
        return Commands.none();
    }

    public BooleanSupplier isJammed() {
        return currLimitHasGP().and(hasGamepieceSupplierRawTOF().not());
    }

    public BooleanSupplier currLimitHasGP() {
        return () -> MathUtils.average(inputsLeft.currentAmps, inputsRight.currentAmps) - GrondConstants.TalonFX.JAM_STATOR >= -5;
    }

    @CreateState("vbus_forward")
    public void infeedVbus() {
        if (!pivotDown.getAsBoolean() || (tofInputs.range >= GrondConstants.PWFTimeOfFlight.TOF_RANGE_THRESH || tofDebounceTimer.get() < 0.09) /*inputs.currentAmps < GrondConstants.STATOR_LIMIT || currentLimitTimer.get() <= GrondConstants.CURRENT_LIMIT_DELAY_SEC*/) {
            if (MathUtils.average(inputsLeft.currentAmps, inputsRight.currentAmps) >= GrondConstants.STATOR_LIMIT) {
                currentLimitTimer.start();
            } else {
                currentLimitTimer.stop();
                currentLimitTimer.reset();
            }
            if (tofInputs.range < GrondConstants.PWFTimeOfFlight.TOF_RANGE_THRESH) tofDebounceTimer.start();
            else {
                tofDebounceTimer.stop();
                tofDebounceTimer.reset();
            }
            ioleft.setVbus(targetVbusLeft);
            ioright.setVbus(targetVbusRight);
        } else {
            tofDebounceTimer.stop();
            tofDebounceTimer.reset();
            currentLimitTimer.stop();
            currentLimitTimer.reset();
            hasCoral = true;
            state = GrondStates.HOLD;
        }
    }

    @CreateState("unjam")
    public void unjam() {
        ioleft.setVbus(-0.25);
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
        if (!hasCoral) state = GrondStates.OFF;
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
        tokio.updateInputs(tofInputs);
        Logger.processInputs("Ground Infeed/MotorLeft", inputsLeft);
        Logger.processInputs("Ground Infeed/MotorRight", inputsRight);
        Logger.processInputs("Ground Infeed/TOF Sensor", tofInputs);
    }
}
