package frc.robot.subsystems.stick;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SysIDUtil;

public class WhipStickConstants {

    public static final double STATOR_LIMIT = 40;
    public static final double GEARING = 1;
    public static final double CURRENT_LIMIT_DELAY_ALGAE_SEC = 0.5;
    public static final double CURRENT_LIMIT_DELAY_CORAL_SEC = 0.1;

    public static final class Sim {
        public static final DCMotor simGearbox = DCMotor.getVex775Pro(1);
        public static final double MANIP_MOI_KgMSquared = 1;
    }

    public static final class TalonSRX {
        public static final int CAN_ID = 17;
        public static final boolean INVERT = false;
        public static final NeutralMode NEUTRALMODE = NeutralMode.Brake;
    }

    public static final class SparkMax {
        public static final int CAN_ID = 0;
        public static final SparkBaseConfig CONFIG = new SparkMaxConfig().inverted(false);
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
    }

    public static final class TalonFX {
        public static final int CAN_ID = 17;
        public static final boolean USE_FOC = true;
        public static final MotorOutputConfigs CONFIG = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Coast);
        public static final CurrentLimitsConfigs CURR_LIMITS = new CurrentLimitsConfigs().withStatorCurrentLimit(80)
                .withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(false);
    }

    public static final SysIdRoutine.Config sysIDConfig = SysIDUtil.defaultConfig();
}
