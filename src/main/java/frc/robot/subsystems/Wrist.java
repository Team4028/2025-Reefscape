// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.Thread.State;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorStates;

public class Wrist extends SubsystemBase {
  public enum WristStates {
    MOVING, AT_POSITION, HOLD_POSITION, OFF
  }

  private SparkMax wristMotor;

  WristStates state;

  private SparkAbsoluteEncoder absoluteEncoder;

  private PIDController pid;

  private double targetPosition = 0;

  /** Creates a new Wrist. */
  public Wrist() {

    wristMotor = new SparkMax(12, MotorType.kBrushless);

    absoluteEncoder = wristMotor.getAbsoluteEncoder();

    pid = new PIDController(.5, 0, 0);
  }

  public double getAbsoluteEncoderPosition() {
    return absoluteEncoder.getPosition();
  }

  public Command runToPosition(double position) {
    return Commands.runOnce(() -> {
      targetPosition = position;
      state = WristStates.MOVING;
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    switch (state) {
      case MOVING:

        pid.calculate(absoluteEncoder.getPosition(), targetPosition);

        break;

      case AT_POSITION:

        break;

      case HOLD_POSITION:

        break;

      case OFF:

        wristMotor.set(0);

        break;

      default:
      
        break;
    }
  }
}
