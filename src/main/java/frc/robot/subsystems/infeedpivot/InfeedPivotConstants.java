package frc.robot.subsystems.infeedpivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.util.PIDStruct;

public class InfeedPivotConstants {

    public static enum InfeedPivotPositions {
        DOWN(0.0),
        UP(1.57);

        public double posRad;

        InfeedPivotPositions(double posRad) {
            this.posRad = posRad;
        }
    }

    public static final PIDStruct pidConfig = new PIDStruct(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    public static final class TalonFX {
        public static final boolean USE_FOC = true;
        public static final int CAN_ID = 21;
        public static final MotorOutputConfigs motorConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
        public static final CurrentLimitsConfigs currLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(false);
        public static final Slot0Configs pidConfigs = pidConfig.makeSlot0Configs();
        public static final MotionMagicConfigs mmConfigs = pidConfig.makeMMConfigs();
    }

    public static final class Cancoder {
        public static final int CAN_ID = 5;
        public static final CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withSensorDirection(SensorDirectionValue.valueOf(TalonFX.motorConfigs.Inverted.name())));
    }
}
