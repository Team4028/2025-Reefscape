package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.PIDStruct;
import frc.robot.util.SysIDUtil;

public class ClimberConstants {
    public static final boolean USE_FOC = true;

    // actually figure these out!!
    public static final double CLIMBER_LENGTH = 0.0;
    public static final double GEAR_RATIO = 100.0;
    public static final double CLIMBER_OFFSET = 0.0;

    // Would we need this?
    public static final double CLIMBER_POSITION1_RAD = 0.0;

    public static final PIDStruct pidConstants = new PIDStruct(0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0);

    public static final class Sim {
        public static final DCMotor simGearbox = USE_FOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1);
        // get actual value for this:
        public static final double CLIMBER_KgMSquared = 0.2;
    }

    public static final class TalonFX {
        // get actual value for this:
        public static final int MOTOR_ID = 13;

        public static final CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(30)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(30).withSupplyCurrentLimitEnable(true);

    }

    public static final SysIdRoutine.Config sysIDConfig = SysIDUtil.defaultConfig();
}
