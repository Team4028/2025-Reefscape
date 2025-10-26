package frc.robot.subsystems.cage;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.PIDStruct;
import frc.robot.util.SysIDUtil;

public class CageConstants {
    public static final boolean USE_FOC = true;

    // actually figure these out!!
    public static final double CLIMBER_LENGTH = 0.0;
    public static final double GEAR_RATIO = 1.0;
    public static final double CLIMBER_OFFSET = 0.0;
    public static final double MOTOR_CURRENT_LIMIT = 30.0;

    // Would we need this?
   

    public static final PIDStruct pidConstants = new PIDStruct(0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // public static final class Sim {
    //     // public static final DCMotor simGearbox = USE_FOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1);
    //     // get actual value for this:
    //     // public static final double CLIMBER_KgMSquared = 0.2;
    // }

    public static final class TalonFX {
        // get actual value for this:
       public static final int MOTOR_ID = 20;
        

        public static final CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(75)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(30).withSupplyCurrentLimitEnable(false);
        public static final MotorOutputConfigs motorConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive);

    }

    public static final SysIdRoutine.Config sysIDConfig = SysIDUtil.defaultConfig();
}
