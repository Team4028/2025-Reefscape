package frc.robot.subsystems.singulator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SingulatorConstants {

    public static final int LSWITCH_ID = 9;

    public static final class TalonFX {
        public static final MotorOutputConfigs motorConfigs = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
        public static final CurrentLimitsConfigs currLims = new CurrentLimitsConfigs().withStatorCurrentLimit(60).withStatorCurrentLimitEnable(true);
        public static final boolean USE_FOC = true;
        public static final int CAN_ID = 18;
    }
}
