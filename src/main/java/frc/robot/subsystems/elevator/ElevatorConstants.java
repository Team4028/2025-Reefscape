package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SysIDUtil;

public class ElevatorConstants {
    public static final double MOTOR_TO_DRUM_RATIO = 0.833333333;
    public static final int STAGES = 2;

    public static final boolean USE_FOC = true;

    // cascading: 1 : 2 per stage (not base stage)
    public static final double CARRIAGE_MASS_KG = 5;
    public static final double CARRIAGE_GEAR_RATIO = MOTOR_TO_DRUM_RATIO * Math.pow(2, STAGES - 1);
    public static final DCMotor simGearbox = USE_FOC ? DCMotor.getKrakenX60Foc(2) : DCMotor.getKrakenX60(2);
    public static final double DRUM_RADIUS = 0.009;
    public static final double DRUM_MOI_KgMSquared = 0.001;

    public static final double ROT_TO_METRES = 1.5 * DRUM_RADIUS * Math.PI * MOTOR_TO_DRUM_RATIO;

    public static final double MAX_HEIGHT_METERS = 60 * ROT_TO_METRES;


    public static final int LEADER_ID = 15, FOLLOWER_ID = 14;


    public static final Slot0Configs pidConfigs = new Slot0Configs().withKP(1.25).withKI(0).withKD(0)
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKS(0.11895).withKV(0.11526).withKA(0.0031419).withKG(0.19185);

    public static final MotorOutputConfigs leaderConfigs = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    public static final MotorOutputConfigs followerConfigs = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    public static final CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(30)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(30).withSupplyCurrentLimitEnable(true);

    public static final SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitThreshold(55)
        .withForwardSoftLimitEnable(true).withReverseSoftLimitThreshold(5).withReverseSoftLimitEnable(true);

    public static final SysIdRoutine.Config sysIDConfig = SysIDUtil.defaultConfig();
}
