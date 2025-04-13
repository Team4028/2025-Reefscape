package frc.robot.subsystems.groundinfeed;

import java.util.Optional;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;

public class GrondConstants {
    public static final double STATOR_LIMIT = 60;
    public static final double CURRENT_LIMIT_DELAY_SEC = 0.8;

    public static final class PWFTimeOfFlight {
        public static final int CAN_ID = 24;
        public static final double TOF_RANGE_THRESH = 43;
        public static final TimeOfFlight.RangingMode mode = TimeOfFlight.RangingMode.Medium;
        public static final Optional<Double> sampleTime = Optional.empty();
    }

    public static final class TalonFX {
        public static final int CAN_ID = 20;
        public static final boolean USE_FOC = true;
        public static final MotorOutputConfigs motorConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Coast);
        public static final CurrentLimitsConfigs currLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(80)
                .withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(false);
    }
}
