package frc.robot.subsystems.arm;

import java.util.Map;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.bskd.annotations.CreateState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.arm.ArmConstants.ArmSafetyData;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.SysIDUtil;

public class Arm extends SubsystemBase {
    private final ProfiledPIDController pid;
    private final ArmFeedforward armFF;
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private final ArmStateTracker stateTracker;
    private double targetVbus = 0.0, targetVoltage = 0.0, targetPositionRad = 0.0;
    private final Map<Boolean, Map<Direction, Command>> sysIDCommands;
    private boolean hasAlgae;
    private final Elevator parentElevator;

    public Arm(ArmIO io, Elevator parentElevator) {
        this.io = io;
        this.parentElevator = parentElevator;
        pid = ArmConstants.pidConfig.makeProfiledPIDController();
        armFF = ArmConstants.pidConfig.makeArmFeedforward();
        stateTracker = new ArmStateTracker();
        sysIDCommands = SysIDUtil.generateTests(ArmConstants.sysIDConfig, this::runMotorCommand, this);
        io.updateInputs(inputs);
        pid.reset(inputs.armEncoderRad);
    }

    public BooleanSupplier readyToScore() {
        return () -> Math.abs(targetPositionRad - inputs.armEncoderRad) <= ArmConstants.PID_TOLERANCE
                && parentElevator.atTargetPosition().getAsBoolean();
    }

    public Command sysIDTest(boolean dynamic, Direction direction) {
        return sysIDCommands.get(dynamic).get(direction);
    }

    public double calculateArmFFkG() {
        return ArmPhysics.armGravityFF(hasAlgae, inputs.armAngleRad,
                inputs.armMotorVelocityRotPerSec * ArmConstants.PI_2, parentElevator);
    }

    public BooleanSupplier atTargetPosition() {
        return () -> Math.abs(targetPositionRad - inputs.armEncoderRad) <= ArmConstants.PID_TOLERANCE;
    }

    public Command runMotorCommand(double vbus) {
        return runOnce(() -> {
            targetVbus = vbus;
            stateTracker.setStateVBus(vbus);
        });
    }

    public Command runMotorCommand(Voltage volts) {
        return runOnce(() -> {
            targetVoltage = volts.magnitude();
            stateTracker.setStateVoltage(volts.magnitude());
        });
    }

    public Command runToPositionCommand(double positionRad) {
        return runOnce(() -> {
            targetPositionRad = positionRad;
            stateTracker.state = ArmStates.POSITION;
        });
    }

    public Command nudgeCommand(double amount) {
        return runToPositionCommand(targetPositionRad + amount);
    }

    public void setContinuousInput(ArmSafetyData data) {
        if (data.enableContinuousInput())
            pid.enableContinuousInput(data.range()[0], data.range()[1]);
        else
            pid.disableContinuousInput();
    }

    public Command setIsIsDanger(boolean isInDanger) {
        return runOnce(() -> {
            stateTracker.setInDanger(isInDanger, this::setContinuousInput);
        });
    }

    public void pidReset() {
        pid.reset(inputs.armEncoderRad);
    }

    @Override
    public void periodic() {
        stateTracker.state.execute(this);
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
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
        io.setVoltage(pid.calculate(inputs.armEncoderRad, stateTracker.safeClampRange(targetPositionRad))
                + armFF.calculate(inputs.armAngleRad, pid.getSetpoint().velocity));
    }

    public double getSimAngle() {
        return inputs.armAngleRad;
    }
}