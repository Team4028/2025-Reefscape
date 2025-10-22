package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.PIDStruct;
import frc.robot.util.SysIDUtil;

public class ClimberConstants {
    public static final boolean USE_FOC = true;

    public static final double CLIMBER_LENGTH = 0.0;
    public static final double GEAR_RATIO = 48.0;
    public static final double CLIMBER_OFFSET = 0.0;
    public static final double TOLERANCE = Units.degreesToRadians(0);


    // top = 1
    // bot = -0.28

    public enum ClimberPositions {
        ACQUIRE(1),
        INTERMED(-0.3),
        CLIMB(-0.75);

        public final double posRad;

        ClimberPositions(double posRad) {
            this.posRad = posRad;
        }
    }

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
                .withStatorCurrentLimit(50)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(30).withSupplyCurrentLimitEnable(true);

    }

    public static final SysIdRoutine.Config sysIDConfig = SysIDUtil.defaultConfig();


    public static final class Cancoder {
        public static final int CAN_ID = 13;
        public static final CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withMagnetOffset(0)
                        .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
    }
}
