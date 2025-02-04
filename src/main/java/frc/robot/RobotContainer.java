// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        private final SendableChooser<Command> autoChooser;
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(edu.wpi.first.units.Units.MetersPerSecond); // kSpeedAt12Volts
                                                                                                                // desired

        private final SlewRateLimiter xLimiter, yLimiter, thetaLimiter;
        // top
        // speed
        private double MaxAngularRate = 2 * Arm.PI_2;
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        // The robot's subsystems and commands are defined here...
        private final Elevator elevator = new Elevator();
        private final Arm arm = new Arm(elevator);
        private final Coral coral = new Coral();
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        // Replace with CommandPS4Controller or CommandJoystick if needed
        private final Telemetry logger = new Telemetry(MaxSpeed);
        private final CommandXboxController driverController = new CommandXboxController(
                        OperatorConstants.kDriverControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                xLimiter = new SlewRateLimiter(2);
                yLimiter = new SlewRateLimiter(2);
                thetaLimiter = new SlewRateLimiter(4);
                NamedCommands.registerCommand("Guarentee Stop", realDrivetrainStop());
                NamedCommands.registerCommand("L4 Score",
                                runToL4().andThen(Commands.waitUntil(arm.atTargetPosition())));
                NamedCommands.registerCommand("Stow",
                                runToStow());
                NamedCommands.registerCommand("Acquire",
                                runAquire()
                                                .andThen(coral.runMotorCommand(.7)
                                                                .alongWith(Commands.waitUntil(
                                                                                coral.hasGamePieceSupplier()))
                                                                .andThen(coral.runMotorCommand(0))));
                NamedCommands.registerCommand("L3 Score",
                                runToL3().andThen(Commands.waitUntil(arm.readyToScore())));
                NamedCommands.registerCommand("Score Outfeed",
                                Commands.waitUntil(arm.readyToScore()).andThen(Commands.waitSeconds(0.5)).andThen(coral.runMotorCommand(-.8).alongWith(Commands.waitSeconds(1))
                                                .andThen(coral.runMotorCommand(0))));
                NamedCommands.registerCommand("Stow Arm", arm.runToPositionCommand(Units.degreesToRadians(180)));
                autoChooser = AutoBuilder.buildAutoChooser();

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

                // Schedule `exampleMethodCommand` when the Xbox controller's B button is
                // pressed,
                // cancelling on release.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(
                                                () -> drive.withVelocityX(
                                                                scaleDriverController(-driverController.getLeftY(),
                                                                                yLimiter, 0.25)
                                                                                * MaxSpeed) // Drive
                                                                // forward
                                                                // with
                                                                // negative Y (forward)
                                                                .withVelocityY(scaleDriverController(
                                                                                -driverController.getLeftX(), xLimiter,
                                                                                0.25)
                                                                                * MaxSpeed) // Drive
                                                                                            // left
                                                                                            // with
                                                                                            // negative
                                                                                            // X
                                                                                            // (left)
                                                                .withRotationalRate(
                                                                                scaleDriverController(-driverController
                                                                                                .getRightX(),
                                                                                                thetaLimiter, 0.25)
                                                                                                * MaxAngularRate) // Drive
                                                                                                                  // counterclockwise
                                // with negative
                                // X (left)
                                ));
                // ====================================================================================================================
                // ====================================================================================================================
                // GABE STUK PLEASE EXPLAIN WHAT HAPPENED -- idk
                // ====================================================================================================================
                // ====================================================================================================================

                // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
                // driverController.b().whileTrue(drivetrain
                // .applyRequest(() -> point
                // .withModuleDirection(
                // new Rotation2d(-driverController.getLeftY(),
                // -driverController.getLeftX()))));

                // // Run SysId routines when holding back/start and X/Y.
                // // Note that each routine should be run exactly once in a single log.
                // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

                // ====================================================================================================================//
                // ====================================================================================================================
                // ====================================================================================================================

                // // reset the field-centric heading on left bumper press
                driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // driverController.b().onTrue(elevator.quasiStaticTest(Direction.kForward));
                // driverController.x().onTrue(elevator.dynamicTest(Direction.kReverse));
                // driverController.a().onTrue(elevator.runToPosition(2));

                // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

                // Schedule `exampleMethodCommand` when the Xbox controller's B button is
                // pressed,
                // // cancelling on release.
                // driverController.povLeft().whileTrue(elevator.quasiStaticTest(Direction.kForward));
                // driverController.povRight().whileTrue(elevator.quasiStaticTest(Direction.kReverse));
                // driverController.povUp().whileTrue(elevator.dynamicTest(Direction.kForward));
                // driverController.povDown().whileTrue(elevator.dynamicTest(Direction.kReverse));
                // driverController.a().onTrue(arm.runToPositionCommand(Math.PI));
                // driverController.b().onTrue(elevator.runToPosition(40));
                // driverController.y().onTrue(elevator.runToPosition(10));
                // driverController.x().onTrue(elevator.reefStateChangeCommand());

                // driverController.rightBumper().onTrue(elevator.runMotorsCommand(0.4)).onFalse(elevator.runMotorsCommand(0));
                // driverController.leftBumper().onTrue(elevator.runMotorsCommand(-0.4)).onFalse(elevator.runMotorsCommand(0));

                // driverController.rightBumper().onTrue(elevator.reefCountChange(1));
                // driverController.leftBumper().onTrue(elevator.reefCountChange(-1));
                // driverController.b().onTrue(elevator.runToReefCount());

                // driverController.leftTrigger().onTrue(elevator.runVoltageCommand(1)).onFalse(elevator.runMotorsCommand(0));
                // driverController.leftBumper().onTrue(elevator.runVoltageCommand(-1)).onFalse(elevator.runMotorsCommand(0));

                // Run to L4
                driverController.x().onTrue(runToL4());
                // Run to L3
                driverController.a().onTrue(runToL3());
                // Run to L2
                driverController.b().onTrue(runToL2());
                // Acquire
                driverController.y().onTrue(runAquire());

                driverController.rightBumper().onTrue(runToStow());

                driverController.leftTrigger().onTrue(coral.runMotorCommand(.7)).onFalse(coral.runMotorCommand(0));
                driverController.leftBumper().onTrue(coral.runMotorCommand(-.7)).onFalse(coral.runMotorCommand(0));

                drivetrain.registerTelemetry(logger::telemeterize);
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        public void resetArmPid() {
                arm.pidReset();
        }

        public Command runArmMotorOff() {
                return arm.runMotorOffCommand();
        }

        public BooleanSupplier armAndElevatorKosher() {
                return () -> arm.atTargetPosition().getAsBoolean() && elevator.atTargetPosition().getAsBoolean();
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
                // An example command will be run in autonomous
        }

        public Command runToL4() {
                return elevator.runToPosition(62).alongWith(arm.runToPositionCommand(Units.degreesToRadians(125.)))
                                .alongWith(Commands.waitUntil(armAndElevatorKosher()));
        }

        public Command runToL3() {
                return elevator.runToPosition(56).alongWith(arm.runToPositionCommand(Units.degreesToRadians(55)))
                                .alongWith(Commands.waitUntil(armAndElevatorKosher()));
        }

        public Command runToL2() {
                return elevator.runToPosition(40.5).alongWith(arm.runToPositionCommand(Units.degreesToRadians(55)))
                                .alongWith(Commands.waitUntil(armAndElevatorKosher()));
        }

        public Command runAquire() {
                return elevator.runToPosition(16.5).alongWith(arm.runToPositionCommand(Units.degreesToRadians(235)))
                                .alongWith(Commands.waitUntil(armAndElevatorKosher()));
        }

        public Command runToStow() {
                return elevator.runToPosition(8).alongWith(arm.runToPositionCommand(Units.degreesToRadians(180)))
                                .alongWith(Commands.waitUntil(armAndElevatorKosher()));
        }

        private double scaleDriverController(double controllerInput, SlewRateLimiter limiter, double baseSpeedPercent) {
                return limiter.calculate(
                                controllerInput * (baseSpeedPercent
                                                + driverController.getRightTriggerAxis() * (1 - baseSpeedPercent)));
        }

        public Command realDrivetrainStop() {
                return drivetrain
                                .runOnce(() -> drivetrain.setControl(
                                                drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)));
        }
}
