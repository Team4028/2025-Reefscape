package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.GetMotorData;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX leader = new TalonFX(ElevatorConstants.TalonFX.LEADER_ID);
    private final StatusSignal<Angle> positionRotLeader = leader.getPosition();
    private final StatusSignal<AngularVelocity> velocityRotPerSecLeader = leader.getVelocity();
    private final StatusSignal<AngularAcceleration> accelerationRotPerSecPerSecLeader = leader.getAcceleration();
    private final StatusSignal<Voltage> appliedVoltageLeader = leader.getMotorVoltage();
    private final StatusSignal<Current> currentAmpsLeader = leader.getSupplyCurrent();

    ;

    private final MotionMagicVoltage pidControl = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(ElevatorConstants.USE_FOC);
    private final DutyCycleOut vbusControl = new DutyCycleOut(0).withEnableFOC(ElevatorConstants.USE_FOC);

    public ElevatorIOTalonFX() {
        leader.getConfigurator().apply(ElevatorConstants.TalonFX.leaderConfigs);

        leader.getConfigurator().apply(ElevatorConstants.TalonFX.pidConfigs);
        leader.getConfigurator().apply(ElevatorConstants.TalonFX.mmConfigs);
        //leader.getConfigurator().apply(ElevatorConstants.TalonFX.softLimits);

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(positionRotLeader, velocityRotPerSecLeader, appliedVoltageLeader,
                currentAmpsLeader);
        inputs.leaderPosition = positionRotLeader.getValueAsDouble();
        inputs.leaderVelocity = velocityRotPerSecLeader.getValueAsDouble();
        inputs.leaderAcceleration = accelerationRotPerSecPerSecLeader.getValueAsDouble();
        inputs.leaderAppliedVolts = appliedVoltageLeader.getValueAsDouble();
        inputs.leaderCurrentAmps = currentAmpsLeader.getValueAsDouble();
        inputs.elevatorPositionInches = inputs.leaderPosition * ElevatorConstants.ROT_TO_IN;
        inputs.elevatorVelocityInchesPerSecond = inputs.leaderVelocity * ElevatorConstants.ROT_TO_IN;
        inputs.leaderData = GetMotorData.getTalonFXData(leader);
    }

    @Override
    public void setVoltage(double volts) {
        leader.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setVbus(double vBus) {
        leader.setControl(vbusControl.withOutput(vBus));
    }

    @Override
    public void setPid(double positionInches) {
        leader.setControl(pidControl.withPosition(positionInches / ElevatorConstants.ROT_TO_IN));
    }
}
