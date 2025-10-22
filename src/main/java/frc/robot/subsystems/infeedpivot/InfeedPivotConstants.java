package frc.robot.subsystems.infeedpivot;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.util.PIDStruct;

public class InfeedPivotConstants {
    public static final double GEAR_RATIO = (64.0 / 12.0) * (54.0 / 28.0) * (32.0 / 22.0);
    public static final PIDStruct pidConfig = new PIDStruct(2.5, 0, 0, 64, 128, 256, 0, 0, 0.065, 1.7, 0, 0);

    public enum InfeedPivotPositions {
        DOWN(0.2),
        PID_CHECK(0.28 + Units.degreesToRadians(1)), // 3 degrees noticable Δ
        HANDOFF(1),
        UP(1.7),
        CLIMB(2.2);

        public final double posRad;

        InfeedPivotPositions(double posRad) {
            this.posRad = posRad;
        }
    }

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

    public static final class TalonFXCC {
        public static final PIDStruct pid = new PIDStruct(8, 0, 4, 80, 300, 1200, 0, 0, 0.1, 3, 0, 0);
        public static final boolean USE_FOC = true;
        public static final int CAN_ID = 9;
        public static final MotorOutputConfigs motorConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
        public static final CurrentLimitsConfigs currLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(90).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(false);
        public static final SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs()
                .withReverseSoftLimitThreshold(0 / ArmConstants.PI_2).withReverseSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(2.7 / ArmConstants.PI_2).withForwardSoftLimitEnable(true);
        public static final Slot0Configs pidConfigs = pid.makeSlot0Configs(GravityTypeValue.Arm_Cosine);
        public static final MotionMagicConfigs mmConfigs = pid.makeMMConfigs();
        public static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withFeedbackRemoteSensorID(Cancoder.CAN_ID)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withRotorToSensorRatio(GEAR_RATIO);
    }

    public static final class Cancoder {
        public static final int CAN_ID = 9;
        public static final CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withMagnetOffset(-0.425048828125)
                        .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
    }
}
