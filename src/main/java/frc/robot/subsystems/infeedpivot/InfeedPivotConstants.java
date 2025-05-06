package frc.robot.subsystems.infeedpivot;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.util.PIDStruct;

public class InfeedPivotConstants {
    public static final double GEAR_RATIO = (64.0 / 12.0) * (54.0 / 28.0) * (32.0 / 22.0);

    public enum InfeedPivotPositions {
        DOWN(0.2),
        HANDOFF(1),
        UP(1.7),
        CLIMB(2.5);

        public final double posRad;

        InfeedPivotPositions(double posRad) {
            this.posRad = posRad;
        }
    }

    public static final PIDStruct pidConfig = new PIDStruct(2.5, 0, 0, 64, 128, 256, 0, 0, 0.065, 1.7, 0, 0);

    public static final class TalonFX {
        public static final boolean USE_FOC = true;
        public static final int CAN_ID = 9;
        public static final MotorOutputConfigs motorConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
        public static final CurrentLimitsConfigs currLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(90).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(false);
        public static final SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs()
                .withReverseSoftLimitThreshold(-0.06 / ArmConstants.PI_2 * GEAR_RATIO).withReverseSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(1.9 / ArmConstants.PI_2 * GEAR_RATIO).withForwardSoftLimitEnable(true);
    }

    public static final class Cancoder {
        public static final int CAN_ID = 9;
        public static final CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withMagnetOffset(-0.425048828125)
                        .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
    }
}
