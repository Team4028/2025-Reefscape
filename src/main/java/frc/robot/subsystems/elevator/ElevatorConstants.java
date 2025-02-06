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
import frc.robot.util.PIDStruct;
import frc.robot.util.SysIDUtil;

public class ElevatorConstants {
    public static final double MOTOR_TO_DRUM_RATIO = 0.0833333333;
    public static final int STAGES = 2;

    public static final double SAFETY_THRESHOLD = 20;

    public static final boolean USE_FOC = true;

    public static final PIDStruct pidConstants = new PIDStruct(0.4, 0, 0, 3, 6, 0.05, 0.6, 0.11512, 0.0029619);
    public static final PIDStruct simPidConstants = new PIDStruct(0.4, 0, 0, 3, 6, 0.05, 0.6, 0.11512, 0.0029619);
    // public static final PIDStruct simPidConstants = new PIDStruct(5, 0, 0, 3, 6, 0, 0, 0, 0);

    // cascading: 1 : 2 per stage (not base stage)
    public static final double CARRIAGE_MASS_Kg = 3.628 / 9.8;
    public static final double CARRIAGE_GEAR_RATIO = MOTOR_TO_DRUM_RATIO * Math.pow(2, STAGES - 1);
    public static final DCMotor simGearbox = USE_FOC ? DCMotor.getKrakenX60Foc(2) : DCMotor.getKrakenX60(2);
    public static final double DRUM_RADIUS_IN = 0.875;
    public static final double DRUM_MOI_KgMSquared = 0.1;

    public static final double ROT_TO_IN = 4 * DRUM_RADIUS_IN * Math.PI * MOTOR_TO_DRUM_RATIO;

    public static final double MAX_HEIGHT_INCHES = 65.5 * ROT_TO_IN;

    public static final double PID_TOLERANCE = 9.1629;

    public static final class TalonFX {
        public static final int LEADER_ID = 15, FOLLOWER_ID = 14;

        public static final Slot0Configs pidConfigs = pidConstants.makeSlotConfigs(GravityTypeValue.Elevator_Static);

        public static final MotorOutputConfigs leaderConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        public static final MotorOutputConfigs followerConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        public static final CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(30)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(30).withSupplyCurrentLimitEnable(true);

        public static final SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(58)
                .withForwardSoftLimitEnable(true).withReverseSoftLimitThreshold(5).withReverseSoftLimitEnable(true);
    }

    public static final SysIdRoutine.Config sysIDConfig = SysIDUtil.defaultConfig();
}
