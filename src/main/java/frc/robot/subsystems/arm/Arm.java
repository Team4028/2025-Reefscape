package frc.robot.subsystems.arm;

import java.util.Map;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Armistice.ArmisticePositions;
import frc.robot.util.MathUtils;
import frc.robot.util.SysIDUtil;
import frc.robot.util.LoggedTunables.LoggedChangableBoolean;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private final ArmStateTracker stateTracker;
    private final LoggedChangableBoolean isBrake = new LoggedChangableBoolean("ArmInBrake", true);
    private double targetVbus = 0.0, targetVoltage = 0.0;
    @AutoLogOutput
    private double targetPositionRad = ArmisticePositions.STOW.getArmPositionRad(0);
    private final Map<Boolean, Map<Direction, Command>> sysIDCommands;
    private boolean hasAlgae;

    public Arm(ArmIO io) {
        this.io = io;
        stateTracker = new ArmStateTracker();
        sysIDCommands = SysIDUtil.generateTests(ArmConstants.sysIDConfig, this::runMotor, this);
        io.updateInputs(inputs);
    }

    public double getVelocityRad() {
        return inputs.armVelocityRotPerSec * 2 * Math.PI;
    }

    public void setPosition(double positionRad) {
        if (io instanceof ArmIOCancoderTalonFX iocanc)
            iocanc.setPosition(positionRad);
    }

    public void configureBrake(boolean isBrake) {
        if (io instanceof ArmIOCancoderTalonFX canio) {
            canio.setBrake(isBrake);
        }
    }

    public Command sysIDTest(boolean dynamic, Direction direction) {
        return sysIDCommands.get(dynamic).get(direction);
    }

    public double calculateArmFFkG(double elevatorAcceleration) {
        return ArmPhysics.armGravityFF(hasAlgae, inputs.armAngleRad,
                inputs.armMotorVelocityRotPerSec * ArmConstants.PI_2, elevatorAcceleration);
    }

    public BooleanSupplier atTargetPosition() {
        return () -> Math.abs(targetPositionRad - inputs.armEncoderRad) <= ArmConstants.PID_TOLERANCE;
    }

    public void runMotor(double vbus) {
        targetVbus = vbus;
        stateTracker.setStateVBus(vbus);
    }

    public void runMotor(Voltage volts) {
        targetVoltage = volts.magnitude();
        stateTracker.setStateVoltage(volts.magnitude());
    }

    public void runToPosition(double positionRad) {
        targetPositionRad = positionRad;
        stateTracker.state = ArmStates.POSITION;
    }

    public void setArmAccel(double accel) {
        if (io instanceof ArmIOCancoderTalonFX iocan) {
            iocan.setArmAccel(accel);
        } else if (io instanceof ArmIOCancoderSourceTalonFX iocans) {
            iocans.setArmAccel(accel);
        }
    }

    public void nudge(double amount) {
        targetPositionRad += amount;
        stateTracker.state = ArmStates.POSITION;
    }

    // public void setContinuousInput(ArmSafetyData data) {
    //     if (data.enableContinuousInput())
    //         pid.enableContinuousInput(data.range()[0], data.range()[1]);
    //     else
    //         pid.disableContinuousInput();
    // }

    // public void pidReset() {
    //     pid.reset(inputs.armEncoderRad);
    // }

    public double getTargetPosition() {
        return targetPositionRad;
    }

    public double getCurrentPosition() {
        return inputs.armEncoderRad;
    }


    public double getCanMagPosition() {
        return inputs.canMagPosition; //working
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        LoggedChangableBoolean.ifChanged(hashCode(), bools -> configureBrake(bools[0]), isBrake);
        Logger.processInputs("Arm", inputs);
        Logger.recordOutput("Arm/ArmEncoderRadFriendly", MathUtils.roundToPlace(inputs.armEncoderRad, 3));
        Logger.recordOutput("Arm/ArmEncoderRawFriendly", MathUtils.roundToPlace(inputs.armEncoderRaw, 3));
        stateTracker.state.execute(this);
    }

    @CreateState("vbus_forward")
    @CreateState("vbus_reverse")
    public void runTargetVBus() {
        io.setVBus(targetVbus);
    }

    @CreateState("off")
    public void stop() {
        io.setVBus(0);
    }

    @CreateState("voltage_forward")
    @CreateState("voltage_reverse")
    public void runTargetVoltage() {
        io.setVoltage(targetVoltage);
    }

    @CreateState("position")
    public void runTargetPosition() {
        io.setPID(targetPositionRad);
    }

    public double getSimAngle() {
        return inputs.armAngleRad;
    }
}