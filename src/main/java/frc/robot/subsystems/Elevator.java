// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Type;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Elevator extends SubsystemBase {

  public enum ElevatorStates {
    IDLE, PREPARE_TO_MOVE, MOVING, HOLDING_POSITION
  }

  private TalonFX motor;

  private RelativeEncoder encoder;

  private PositionVoltage pidControl;

  private ElevatorStates state;

  private double targetVbus = 0.0;

  private double targetPostition = 0;

  private SysIdRoutine.Config sysIDConfig;
  private SysIdRoutine.Mechanism sysIDMech;
  private SysIdRoutine sysId;

  /** Creates a new Elevator. */
  public Elevator() {

    motor = new TalonFX(0);

    sysIDConfig = new SysIdRoutine.Config();

    sysIDMech = new SysIdRoutine.Mechanism(v -> motor.setVoltage(v.magnitude()), null, this);
    
    sysId = new SysIdRoutine(sysIDConfig, sysIDMech);

    motor.getConfigurator().apply(new Slot0Configs().withKP(0).withKI(0).withKD(0));

    pidControl = new PositionVoltage(0).withEnableFOC(true).withSlot(0);

  };

  public Command runToPosition(double position) {
    return Commands.runOnce(() -> {
      targetPostition = position;
      state = ElevatorStates.PREPARE_TO_MOVE;
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

        break;
      case PREPARE_TO_MOVE:
        if (motor.getPosition().getValueAsDouble() < targetPostition) {
          motor.getConfigurator().apply(new Slot0Configs().withKP(.5).withKI(0).withKD(0));

        } else if (motor.getPosition().getValueAsDouble() > targetPostition) {
          motor.getConfigurator().apply(new Slot0Configs().withKP(.5).withKI(0).withKD(0));
        }
        state = ElevatorStates.MOVING;
        break;

      case MOVING:
        motor.setControl(pidControl.withPosition(targetPostition));
        break;

      case HOLDING_POSITION:

        break;

      default:
        break;
    }

  }
}