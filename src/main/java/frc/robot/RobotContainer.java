// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Elevator elevator = new Elevator();
  private final Arm arm = new Arm(elevator);
  private final Coral coral = new Coral();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // driverController.povLeft().whileTrue(elevator.quasiStaticTest(Direction.kForward));
    // driverController.povRight().whileTrue(elevator.quasiStaticTest(Direction.kReverse));
    // driverController.povUp().whileTrue(elevator.dynamicTest(Direction.kForward));
    // driverController.povDown().whileTrue(elevator.dynamicTest(Direction.kReverse));
    // driverController.a().onTrue(elevator.runToPosition(53));
    // driverController.y().onTrue(elevator.runToPosition(7));
    // driverController.x().onTrue(elevator.reefStateChangeCommand());
    // driverController.rightBumper().onTrue(elevator.runMotorsCommand(0.65)).onFalse(elevator.runMotorsCommand(0));
    // driverController.leftBumper().onTrue(elevator.runMotorsCommand(-0.65)).onFalse(elevator.runMotorsCommand(0));

    // driverController.rightBumper().onTrue(elevator.reefCountChange(1));
    // driverController.leftBumper().onTrue(elevator.reefCountChange(-1));
    // driverController.b().onTrue(elevator.runToReefCount());

    // driverController.rightTrigger().onTrue(elevator.runVoltageCommand(1)).onFalse(elevator.runMotorsCommand(0));
    // driverController.leftTrigger().onTrue(elevator.runVoltageCommand(-1)).onFalse(elevator.runMotorsCommand(0));

    // driverController.povRight().onTrue(arm.runToPositionCommand(Math.PI / 4));
    // driverController.povUp().onTrue(arm.runToPositionCommand(3 * Math.PI / 4));
    // driverController.povLeft().onTrue(arm.runToPositionCommand(5 * Math.PI / 4));
    // driverController.povDown().onTrue(arm.runToPositionCommand((7 * Math.PI) / 4));
    
    
    // driverController.a().onTrue(Commands.runOnce(() -> arm.setInDanger(!arm.isInDanger())));
    
    driverController.x().onTrue(coral.runMotorCommand(.7)).onFalse(coral.runMotorCommand(0));
    driverController.b().onTrue(coral.runMotorCommand(-.6)).onFalse(coral.runMotorCommand(0));

  }

  public void resetArmPid() {
    arm.pidReset();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
