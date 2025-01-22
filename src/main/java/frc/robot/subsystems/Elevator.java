// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SysIDUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Elevator extends SubsystemBase {

  public enum ElevatorStates {
    IDLE, PREPARE_TO_MOVE, MOVING_POSITION, HOLDING_POSITION, MOVING_VBUS, MOVING_VOLTAGE,
  }

  private final TalonFX leader;
  private final TalonFX follower;

  private final PositionVoltage pidControl;
  private final VoltageOut voltageControl;
  private final DutyCycleOut vbusControl;

  private ElevatorStates state;

  private double targetVbus = 0.0, targetPostition = 0.0, targetVoltage = 0.0;

  private SysIdRoutine.Config sysIDConfig;
  private SysIdRoutine.Mechanism sysIDMech;
  private SysIdRoutine sysId;

  public static final class ElevatorConstants {

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
  }

  /** Creates a new Elevator. */
  public Elevator() {

    leader = new TalonFX(ElevatorConstants.LEADER_ID);
    follower = new TalonFX(ElevatorConstants.FOLLOWER_ID);
    leader.getConfigurator().apply(ElevatorConstants.leaderConfigs);
    follower.getConfigurator().apply(ElevatorConstants.followerConfigs);

    state = ElevatorStates.IDLE;

    sysIDConfig = new SysIdRoutine.Config(null, null, null, SysIDUtil::logSysIdState);

    pidControl = new PositionVoltage(0).withEnableFOC(true).withSlot(0);
    voltageControl = new VoltageOut(0);
    vbusControl = new DutyCycleOut(0);

    sysIDMech = new SysIdRoutine.Mechanism(v -> {
      leader.setControl(voltageControl.withOutput(v));
    }, null, this);

    sysId = new SysIdRoutine(sysIDConfig, sysIDMech);

    leader.getConfigurator().apply(ElevatorConstants.pidConfigs);

    leader.getConfigurator().apply(ElevatorConstants.currentLimitConfigs);
    follower.getConfigurator().apply(ElevatorConstants.currentLimitConfigs);

    leader.getConfigurator().apply(ElevatorConstants.softLimits);
    follower.getConfigurator().apply(ElevatorConstants.softLimits);

  };

  public Command setNeutralMode(NeutralModeValue value) {
    return runOnce(() -> {
      var config = new MotorOutputConfigs().withNeutralMode(value);
      leader.getConfigurator().apply(config);
      follower.getConfigurator().apply(config);
    });

  }

  public double getAccelaration() {
    return leader.getAcceleration().getValueAsDouble();
  }

  public Command runToPosition(double position) {
    return runOnce(() -> {
      targetPostition = position;
      state = ElevatorStates.PREPARE_TO_MOVE;
    });
  }

  public Command runMotorsCommand(double vbus) {
    return runOnce(() -> {
      state = vbus == 0 ? ElevatorStates.IDLE : ElevatorStates.MOVING_VBUS;
      targetVbus = vbus;
    });
  }

  public Command runVoltageCommand(double voltage) {
    return runOnce(() -> {
      state = ElevatorStates.MOVING_VOLTAGE;
      targetVoltage = voltage;
    });
  }

  public Command quasiStaticTest(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command dynamicTest(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  // Use addRequirements() here to declare subsystem dependencies.

  // Called when the command is initially scheduled.

  @Override
  public void periodic() {
    switch (state) {
      case IDLE:
        leader.setControl(vbusControl.withOutput(0));
        break;
      case PREPARE_TO_MOVE:
        state = ElevatorStates.MOVING_POSITION;
        break;

      case MOVING_POSITION:
        leader.setControl(pidControl.withPosition(targetPostition));
        break;

      case HOLDING_POSITION:
        break;
      case MOVING_VBUS:
        leader.setControl(vbusControl.withOutput(targetVbus));
        break;
      case MOVING_VOLTAGE:
        leader.setControl(voltageControl.withOutput(targetVoltage));
        break;
    }

    follower.setControl(new StrictFollower(15));

    SmartDashboard.putNumber("LMotor Position", leader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("LMotor Velocity", leader.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("LMotorAmperes", leader.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("RMotorAmperes", follower.getSupplyCurrent().getValueAsDouble());
  }
}