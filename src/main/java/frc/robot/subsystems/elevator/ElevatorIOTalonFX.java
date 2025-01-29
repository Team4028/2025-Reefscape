package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX leader = new TalonFX(ElevatorConstants.LEADER_ID);
    private final TalonFX follower = new TalonFX(ElevatorConstants.FOLLOWER_ID);
    private final StatusSignal<Angle> positionRotLeader = leader.getPosition();
    private final StatusSignal<AngularVelocity> velocityRotPerSecLeader = leader.getVelocity();
    private final StatusSignal<AngularAcceleration> accelerationRotPerSecPerSecLeader = leader.getAcceleration();
    private final StatusSignal<Voltage> appliedVoltageLeader = leader.getMotorVoltage();
    private final StatusSignal<Current> currentAmpsLeader = leader.getSupplyCurrent();
    private final StatusSignal<Angle> positionRotFollower = follower.getPosition();
    private final StatusSignal<AngularVelocity> velocityRotPerSecFollower = follower.getVelocity();
    private final StatusSignal<AngularAcceleration> accelerationRotPerSecPerSecFollower = follower.getAcceleration();
    private final StatusSignal<Voltage> appliedVoltageFollower = follower.getMotorVoltage();
    private final StatusSignal<Current> currentAmpsFollower = follower.getSupplyCurrent();

    private final PositionVoltage pidControl = new PositionVoltage(0).withEnableFOC(ElevatorConstants.USE_FOC).withSlot(0);
    private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(ElevatorConstants.USE_FOC);
    private final DutyCycleOut vbusControl = new DutyCycleOut(0).withEnableFOC(ElevatorConstants.USE_FOC);
    private final StrictFollower followerControl = new StrictFollower(ElevatorConstants.LEADER_ID);

    public ElevatorIOTalonFX() {
        leader.getConfigurator().apply(ElevatorConstants.leaderConfigs);
        follower.getConfigurator().apply(ElevatorConstants.followerConfigs);
        leader.getConfigurator().apply(ElevatorConstants.pidConfigs);
        leader.getConfigurator().apply(ElevatorConstants.softLimits);
        follower.getConfigurator().apply(ElevatorConstants.softLimits);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(positionRotLeader, positionRotFollower, velocityRotPerSecLeader,
                velocityRotPerSecFollower, appliedVoltageLeader, appliedVoltageFollower, currentAmpsLeader,
                currentAmpsFollower);

        inputs.leaderPosition = positionRotLeader.getValueAsDouble();
        inputs.followerPosition = positionRotFollower.getValueAsDouble();
        inputs.leaderVelocity = velocityRotPerSecLeader.getValueAsDouble();
        inputs.followerVelocity = velocityRotPerSecFollower.getValueAsDouble();
        inputs.leaderAcceleration = accelerationRotPerSecPerSecLeader.getValueAsDouble();
        inputs.followerAcceleration = accelerationRotPerSecPerSecFollower.getValueAsDouble();
        inputs.leaderAppliedVolts = appliedVoltageLeader.getValueAsDouble();
        inputs.followerAppliedVolts = appliedVoltageFollower.getValueAsDouble();
        inputs.leaderCurrentAmps = currentAmpsLeader.getValueAsDouble();
        inputs.followerCurrentAmps = currentAmpsFollower.getValueAsDouble();
    }
    
    @Override
    public void setVoltage(double volts) {
        leader.setControl(voltageControl.withOutput(volts));
        follower.setControl(followerControl);
    }

    @Override
    public void setVbus(double vBus) {
        leader.setControl(vbusControl.withOutput(vBus));
        follower.setControl(followerControl);
    }

    @Override
    public void setPid(double positionRot) {
        leader.setControl(pidControl.withPosition(positionRot));
        follower.setControl(followerControl);
    }
}
