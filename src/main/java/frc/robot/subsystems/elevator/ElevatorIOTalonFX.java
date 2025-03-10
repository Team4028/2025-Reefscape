package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.MotorData;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX leader = new TalonFX(ElevatorConstants.TalonFX.LEADER_ID);
    private final TalonFX follower = new TalonFX(ElevatorConstants.TalonFX.FOLLOWER_ID);
    private final StatusSignal<Angle> positionRotLeader = leader.getPosition();
    private final StatusSignal<Angle> positionRotFollower = follower.getPosition();
    private final StatusSignal<AngularVelocity> velocityRotPerSecLeader = leader.getVelocity();
    private final StatusSignal<AngularVelocity> velocityRotPerSecFollower = follower.getVelocity();
    private final StatusSignal<AngularAcceleration> accelerationRotPerSecPerSecLeader = leader.getAcceleration();
    private final StatusSignal<AngularAcceleration> accelerationRotPerSecPerSecFollower = follower.getAcceleration();
    private final StatusSignal<Voltage> appliedVoltageLeader = leader.getMotorVoltage();
    private final StatusSignal<Voltage> appliedVoltageFollower = follower.getMotorVoltage();
    private final StatusSignal<Current> currentAmpsLeader = leader.getStatorCurrent();
    private final StatusSignal<Current> currentAmpsFollower = follower.getStatorCurrent();

    private final MotionMagicVoltage pidControl = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(ElevatorConstants.USE_FOC);
    private final DutyCycleOut vbusControl = new DutyCycleOut(0).withEnableFOC(ElevatorConstants.USE_FOC);
    private final StrictFollower indenturedServitude = new StrictFollower(ElevatorConstants.TalonFX.LEADER_ID);

    public ElevatorIOTalonFX() {
        BaseStatusSignal.setUpdateFrequencyForAll(20, positionRotFollower, velocityRotPerSecFollower,
                accelerationRotPerSecPerSecFollower, appliedVoltageFollower, currentAmpsFollower);
        BaseStatusSignal.setUpdateFrequencyForAll(10, positionRotLeader, velocityRotPerSecLeader,
                accelerationRotPerSecPerSecLeader, appliedVoltageLeader, currentAmpsLeader);
        leader.optimizeBusUtilization();
        follower.optimizeBusUtilization();
        leader.getConfigurator().apply(ElevatorConstants.TalonFX.leaderConfigs);
        follower.getConfigurator().apply(ElevatorConstants.TalonFX.followerConfigs);
        leader.getConfigurator().apply(ElevatorConstants.TalonFX.pidConfigs);
        leader.getConfigurator().apply(ElevatorConstants.TalonFX.mmConfigs);
        leader.getConfigurator().apply(ElevatorConstants.TalonFX.tcConfigs);
        leader.getConfigurator().apply(ElevatorConstants.TalonFX.softLimits);

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(positionRotLeader, velocityRotPerSecLeader, accelerationRotPerSecPerSecLeader,
                appliedVoltageLeader,
                currentAmpsLeader, positionRotFollower, velocityRotPerSecFollower, accelerationRotPerSecPerSecFollower,
                appliedVoltageFollower, currentAmpsFollower);
        inputs.leaderPosition = positionRotLeader.getValueAsDouble();
        inputs.leaderVelocity = velocityRotPerSecLeader.getValueAsDouble();
        inputs.leaderAcceleration = accelerationRotPerSecPerSecLeader.getValueAsDouble();
        inputs.leaderAppliedVolts = appliedVoltageLeader.getValueAsDouble();
        inputs.leaderCurrentAmps = currentAmpsLeader.getValueAsDouble();
        inputs.followerPosition = positionRotFollower.getValueAsDouble();
        inputs.followerVelocity = velocityRotPerSecFollower.getValueAsDouble();
        inputs.followerAcceleration = accelerationRotPerSecPerSecFollower.getValueAsDouble();
        inputs.followerAppliedVolts = appliedVoltageFollower.getValueAsDouble();
        inputs.followerCurrentAmps = currentAmpsFollower.getValueAsDouble();
        inputs.elevatorPositionInches = inputs.leaderPosition * ElevatorConstants.ROT_TO_IN;
        inputs.elevatorVelocityInchesPerSecond = inputs.leaderVelocity * ElevatorConstants.ROT_TO_IN;
        inputs.leaderData = MotorData.getMotorData(leader);
        inputs.followerData = MotorData.getMotorData(follower);
    }

    @Override
    public void setVoltage(double volts) {
        leader.setControl(voltageControl.withOutput(volts));
        follower.setControl(indenturedServitude);
    }

    @Override
    public void setVbus(double vBus) {
        leader.setControl(vbusControl.withOutput(vBus));
        follower.setControl(indenturedServitude);
    }

    @Override
    public void setPid(double positionInches) {
        leader.setControl(pidControl.withPosition(positionInches / ElevatorConstants.ROT_TO_IN));
        follower.setControl(indenturedServitude);
    }
}
