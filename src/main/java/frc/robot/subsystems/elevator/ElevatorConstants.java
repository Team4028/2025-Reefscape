package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.PIDStruct;
import frc.robot.util.SysIDUtil;

public class ElevatorConstants {
    public static final double MOTOR_TO_DRUM_RATIO = 0.11111111111111111111111111111;

    public static final double SAFETY_THRESHOLD = 20;

    public static final boolean USE_FOC = true;

    public static final DCMotor gearbox = USE_FOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1);

    // public static final PIDStruct pidConstants = new PIDStruct(8, 0, 0, 86, 210, 400, -60, 60, 0.043915, 0.39241,
    //         0.13165, 0.0033318);
    public static final PIDStruct pidConstants = new PIDStruct(8, 0, 0, 100, 400, 1600, -60, 60, 0.043915, 0.39241,
            0.13165, 0.0033318);
    public static final PIDStruct simPidConstants = new PIDStruct(0.4, 0, 0, 118, 254, Constants.THE_BEST_NUMBER, 0, 0,
            0.05, 0.6, 0.11512, 0.0029619);

    // cascading: 1 : 2 per stage (not base stage)
    public static final double CARRIAGE_MASS_Kg = 3.628;
    public static final double DRUM_RADIUS_IN = 0.875; // no longer real, wtvr
    public static final double DRUM_MOI_KgMSquared = 1 * DRUM_RADIUS_IN * DRUM_RADIUS_IN * 0.8;

    public static final double ROT_TO_IN = 0.79174;

    public static final double MAX_HEIGHT_INCHES = 72 * ROT_TO_IN;

    public static final double PID_TOLERANCE = 0.1;// 9.1629;

    // public static final double FORWARD_SOFT_LIMIT_INCHES = 64;
    public static final double FORWARD_SOFT_LIMIT_ROTATIONS = 56 / ROT_TO_IN;
    // the hard stops are at 66 "inches" using bad math which is 72 rotations of the
    // kraken.

    public static final class TalonFX {
        public static final int LEADER_ID = 15, FOLLOWER_ID = 14;

        public static final Slot0Configs pidConfigs = pidConstants.makeSlotConfigs(GravityTypeValue.Elevator_Static);
        public static final MotionMagicConfigs mmConfigs = pidConstants.makeMMConfigs();
        public static final TorqueCurrentConfigs tcConfigs = pidConstants.makeTCConfigs();

        public static final MotorOutputConfigs leaderConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        public static final MotorOutputConfigs followerConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        public static final CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(80)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(80).withSupplyCurrentLimitEnable(true);

        public static final SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT_ROTATIONS)
                .withForwardSoftLimitEnable(true).withReverseSoftLimitThreshold(0.5).withReverseSoftLimitEnable(true);
    }

    public static final SysIdRoutine.Config sysIDConfig = new SysIdRoutine.Config(null, Volts.of(4), null,
            SysIDUtil::logSysIdState);
}
