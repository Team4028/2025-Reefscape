package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.PIDStruct;
import frc.robot.util.SysIDUtil;

public class ArmConstants {

    public static final double ARM_LENGTH_METRES = Units.inchesToMeters(28);
    public static final double ARM_MASS_Kg = 3.63;
    public static final double CG = ARM_LENGTH_METRES / 2; // uniform density
    public static final double GEAR_RATIO = 79.347;
    public static final double PID_TOLERANCE = Units.degreesToRadians(1);
    public static final double SAFE_DISTANCE = Units.degreesToRadians(3);

    public static final boolean USE_FOC = true;

    public static final class Sim {
        public static final DCMotor simGearbox = USE_FOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1);
        public static final double ARM_MOI_KgMSquared = ARM_MASS_Kg * ARM_LENGTH_METRES * ARM_LENGTH_METRES * 0.7;
    }

    public static class TalonFX {
        public static final int MOTOR_ID = 10;
        public static final MotorOutputConfigs motorConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
        public static final Slot0Configs pidConfigs = pidConfig.makeSlotConfigs(GravityTypeValue.Arm_Cosine);
        public static final MotionMagicConfigs mmConfigs = pidConfig.makeMMConfigs();

    }

    public static class SparkEncoder {
        public static final int ENCODER_ID = 9;
        public static final SparkBaseConfig encoderConfig = new SparkMaxConfig()
                .apply(new AbsoluteEncoderConfig().inverted(true));
        public static final double ENCODER_OFFSET = 0.826;
    }

    public static class DIOEncoder {
        public static final int DIO_PIN = 0;
        public static final boolean INVERTED = true;
    }


    public static final double PI_1_2 = 0.5 * Math.PI;
    public static final double PI_3_2 = 1.5 * Math.PI;
    public static final double PI_2 = 2 * Math.PI;
    public static final double ARM_ACCEL_W_ALGAE = 100;

    public static final PIDStruct pidConfig = new PIDStruct(4, 0, 0, 400, 400, 1600, 0, 0.15, 0.425, 0, 0, 0);
    // public static final PIDStruct pidConfig = new PIDStruct(4, 0, 0, 100, 200, 800, 0, 0, 0.3, 0.5, 0, 0);
    public static final PIDStruct simPidConfig = new PIDStruct(16, 0, 0, PI_2, 2 * PI_2, 0, 0, 0, 0.2, 0.5, 0, 0);

    public static final SysIdRoutine.Config sysIDConfig = SysIDUtil.defaultConfig();
}
