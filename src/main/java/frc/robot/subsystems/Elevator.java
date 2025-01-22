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
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Elevator extends SubsystemBase {

  public enum ElevatorStates {
    IDLE, PREPARE_TO_MOVE, MOVING, HOLDING_POSITION, MOVING_VBUS,
  }

  private final TalonFX leader;
  private final TalonFX follower;

  private PositionVoltage pidControl;

  private ElevatorStates state;

  private double targetVbus = 0.0;

  private double targetPostition = 0;

  private SysIdRoutine.Config sysIDConfig;
  private SysIdRoutine.Mechanism sysIDMech;
  private SysIdRoutine sysId;

  /** Creates a new Elevator. */
  public Elevator() {

    leader = new TalonFX(15);
    follower = new TalonFX(14);
    leader.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake));
    follower.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake));

    state = ElevatorStates.IDLE;

    sysIDConfig = new SysIdRoutine.Config();

    sysIDMech = new SysIdRoutine.Mechanism(v -> {
      leader.setVoltage(v.magnitude());
      follower.setVoltage(v.magnitude());
    }, null, this);

    sysId = new SysIdRoutine(sysIDConfig, sysIDMech);

    leader.getConfigurator()
        .apply(new Slot0Configs().withKP(2.0).withKI(0).withKD(0).withGravityType(GravityTypeValue.Elevator_Static));

    var currLimit = new CurrentLimitsConfigs().withStatorCurrentLimit(30).withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(30).withSupplyCurrentLimitEnable(true);
    leader.getConfigurator().apply(currLimit);
    follower.getConfigurator().apply(currLimit);
    var softlimitConfigs = new SoftwareLimitSwitchConfigs().withForwardSoftLimitThreshold(45)
        .withForwardSoftLimitEnable(true).withReverseSoftLimitThreshold(15).withReverseSoftLimitEnable(true);
    leader.getConfigurator().apply(softlimitConfigs);
    follower.getConfigurator().apply(softlimitConfigs);

    pidControl = new PositionVoltage(0).withEnableFOC(true).withSlot(0);

  };

  public Command setNeutralMode(NeutralModeValue value) {
    return runOnce(() -> {
      var config = new MotorOutputConfigs().withNeutralMode(value);
      leader.getConfigurator().apply(config);
      leader.getConfigurator().apply(config);
    });

  }

  public double getAccelaration() {
    return (leader.getAcceleration().getValueAsDouble() + follower.getAcceleration().getValueAsDouble()) / 2;
  }

  public Command runToPosition(double position) {
    return runOnce(() -> {
      targetPostition = position;
      state = ElevatorStates.PREPARE_TO_MOVE;
    });
  }

  public Command runMotors(double vbus) {
    return runOnce(() -> {
      state = vbus == 0 ? ElevatorStates.IDLE : ElevatorStates.MOVING_VBUS;
      targetVbus = vbus;
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
        leader.setControl(new DutyCycleOut(0));
        break;
      case PREPARE_TO_MOVE:
        state = ElevatorStates.MOVING;
        break;

      case MOVING:
        leader.setControl(pidControl.withPosition(targetPostition));
        break;

      case HOLDING_POSITION:
        break;
      case MOVING_VBUS:
        leader.set(targetVbus);
        break;
      default:
        break;
    }

    follower.setControl(new StrictFollower(15));

    SmartDashboard.putNumber("LMotor Position", leader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("LMotor Velocity", leader.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("LMotorAmperes", leader.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("RMotorAmperes", follower.getSupplyCurrent().getValueAsDouble());
  }
}