package frc.robot.subsystems.infeedpivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.util.PIDStruct;

public class InfeedPivotConstants {
    public static final double GEAR_RATIO = 18.857;

    public static enum InfeedPivotPositions {
        DOWN(0.2),
        HANDOFF(1),
        UP(1.7);

        public double posRad;

        InfeedPivotPositions(double posRad) {
            this.posRad = posRad;
        }
    }

    public static final PIDStruct pidConfig = new PIDStruct(2, 0, 0, 64, 128, 256, 0, 0, 0.065, 1.7, 0, 0);
    // public static final PIDStruct pid2Config = new PIDStruct(1, 0, 0, 40, 80, 320, 0, 0, 0, 0, 0, 0);

    public static final class TalonFX {
        public static final boolean USE_FOC = true;
        public static final int CAN_ID = 21;
        public static final MotorOutputConfigs motorConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
        // public static final TalonFXConfiguration feedback = new TalonFXConfiguration()
                // .withFeedback(new FeedbackConfigs().withFeedbackRemoteSensorID(5).withRotorToSensorRatio(GEAR_RATIO)
                //         .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder));
        public static final CurrentLimitsConfigs currLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(false);
        // public static final Slot0Configs pidConfigs = pidConfig.makeSlot0Configs(GravityTypeValue.Arm_Cosine); // 0 - 2
        // public static final Slot1Configs pid2Configs = pid2Config.makeSlot1Configs(GravityTypeValue.Arm_Cosine);
        // public static final MotionMagicConfigs mmConfigs = pidConfig.makeMMConfigs();
        public static final SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs()
                .withReverseSoftLimitThreshold(0 / ArmConstants.PI_2 * GEAR_RATIO).withReverseSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(1.9 / ArmConstants.PI_2 * GEAR_RATIO).withForwardSoftLimitEnable(true);
    }

    public static final class Cancoder {
        public static final int CAN_ID = 5;
        public static final CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withMagnetOffset(-0.425048828125)
                        .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
    }
}
