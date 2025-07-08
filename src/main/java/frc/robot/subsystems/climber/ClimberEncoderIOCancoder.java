package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.arm.ArmConstants;

public class ClimberEncoderIOCancoder implements ClimberIO {
    private final CANcoder cancoder = new CANcoder(ClimberConstants.Cancoder.CAN_ID);
    private final StatusSignal<Angle> position = cancoder.getAbsolutePosition();
    private final StatusSignal<AngularVelocity> velocity = cancoder.getVelocity();

    public ClimberEncoderIOCancoder() {
        cancoder.getConfigurator().apply(ClimberConstants.Cancoder.cancoderConfigs);
        BaseStatusSignal.setUpdateFrequencyForAll(100, position, velocity);

        cancoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(position, velocity);
        inputs.position = position.getValueAsDouble() * ArmConstants.PI_2;
        inputs.velocity = velocity.getValueAsDouble() * ArmConstants.PI_2;
        inputs.connected = cancoder.isConnected();
    }
}
