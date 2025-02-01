// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
  private final TalonSRX motor;
  private InfeedStates state;
  private double targetVbus = 0;
  private boolean hasGamePiece = false;

  /** Creates a new Infeed. */
  public Coral() {
    motor = new TalonSRX(17);
    motor.setInverted(false);
    state = InfeedStates.OFF;
  }

  public enum InfeedStates {
    INFEED,
    OUTFEED,
    OFF
  }

  public BooleanSupplier hasGamePieceSupplier() {
    return () -> hasGamePiece;
  }

  public Command runMotorCommand(double vbus) {
    return runOnce(() -> {
      targetVbus = vbus;
      state = vbus > 0 ? InfeedStates.INFEED : (vbus < 0 ? InfeedStates.OUTFEED : InfeedStates.OFF);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putString("Current Infeed State", state.name());

    switch (state) {
      case OFF:
        motor.set(TalonSRXControlMode.PercentOutput, 0.0);
        break;
      case OUTFEED:
        hasGamePiece = false;
      case INFEED:
        if (motor.getStatorCurrent() < 40)
          motor.set(TalonSRXControlMode.PercentOutput, targetVbus);
        else {
          motor.set(TalonSRXControlMode.PercentOutput, 0);
          hasGamePiece = true;
          state = InfeedStates.OFF;
        }
        break;
    }
  }

  public boolean hasGamePiece() {
    return hasGamePiece;
  }

  
}
