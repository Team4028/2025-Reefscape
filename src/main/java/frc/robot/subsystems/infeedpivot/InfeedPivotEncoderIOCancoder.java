package frc.robot.subsystems.infeedpivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.ArmConstants;

public class InfeedPivotEncoderIOCancoder implements InfeedPivotEncoderIO {
    private final CANcoder cancoder = new CANcoder(InfeedPivotConstants.Cancoder.CAN_ID,
            TunerConstants.DrivetrainConstants.CANBusName);
    private final StatusSignal<Angle> position = cancoder.getAbsolutePosition();
    private final StatusSignal<AngularVelocity> velocity = cancoder.getVelocity();

    public InfeedPivotEncoderIOCancoder() {
        cancoder.getConfigurator().apply(InfeedPivotConstants.Cancoder.cancoderConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(100, position, velocity);
        cancoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(InfeedPivotEncoderIOInputs inputs) {
        BaseStatusSignal.refreshAll(position, velocity);
        inputs.positionRad = position.getValueAsDouble() * ArmConstants.PI_2;
        inputs.velocityRad = velocity.getValueAsDouble() * ArmConstants.PI_2;
        inputs.connected = cancoder.isConnected();
        InfeedPivotEncoderIO.super.updateInputs(inputs);
    }
}
