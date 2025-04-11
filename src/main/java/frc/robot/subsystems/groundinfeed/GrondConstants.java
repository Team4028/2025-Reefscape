package frc.robot.subsystems.groundinfeed;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class GrondConstants {
    public static final double STATOR_LIMIT = 60;
    public static final double CURRENT_LIMIT_DELAY_SEC = 0.8;

    public static final class TalonFX {
        public static final int CAN_ID = 20;
        public static final boolean USE_FOC = true;
        public static final MotorOutputConfigs motorConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
        public static final CurrentLimitsConfigs currLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(80)
                .withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(false);
    }
}
