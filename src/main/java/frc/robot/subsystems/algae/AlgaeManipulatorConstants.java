package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SysIDUtil;

public class AlgaeManipulatorConstants {
    public static final double SUPPLY_LIMIT = 60;
    public static final double GEARING = 10;
    public static final double CURRENT_LIMIT_DELAY_SEC = 0.5;

    public static final class TalonSRX {
        public static final int CAN_ID = 18;
        public static final boolean INVERT = false;

    }

    public static final class TalonFX {
        public static final int CAN_ID = 18;
        public static final MotorOutputConfigs motorConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
    }

    public static final class Sim {
        public static final DCMotor simGearbox = DCMotor.getVex775Pro(1);
        public static final double MANIP_MOI_KgMSquared = 1;
    }

    public static final SysIdRoutine.Config sysIDConfig = SysIDUtil.defaultConfig();
}
