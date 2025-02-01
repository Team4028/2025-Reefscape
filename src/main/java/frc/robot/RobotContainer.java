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
    private double MaxAngularRate = edu.wpi.first.units.Units.RotationsPerSecond.of(0.75)
            .in(edu.wpi.first.units.Units.RadiansPerSecond); // 3/4 of a rotation per
    // second max angular
    // velocity
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
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
        xLimiter = new SlewRateLimiter(4);
        yLimiter = new SlewRateLimiter(4);
        thetaLimiter = new SlewRateLimiter(4);
        NamedCommands.registerCommand("L4 Score",
                elevator.runToPosition(54).alongWith(Commands.waitUntil(elevator.atTargetPosition()))
                        .andThen(arm.runToPositionCommand(edu.wpi.first.math.util.Units.degreesToRadians(123))));

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
                        () -> drive.withVelocityX(xLimiter.calculate(-driverController.getLeftY() * 0.1) * MaxSpeed) // Drive
                                // forward
                                // with
                                // negative Y (forward)
                                .withVelocityY(yLimiter.calculate(-driverController.getLeftX() * 0.1) * MaxSpeed) // Drive
                                                                                                                   // left
                                                                                                                   // with
                                                                                                                   // negative
                                                                                                                   // X
                                                                                                                   // (left)
                                .withRotationalRate(
                                        thetaLimiter.calculate(-driverController.getRightX() * 0.4) * MaxAngularRate) // Drive
                                                                                                                       // counterclockwise
                // with negative
                // X (left)
                ));
// ====================================================================================================================
// ====================================================================================================================
// GABE STUK PLEASE EXPLAIN WHAT HAPPENED
// ====================================================================================================================
// ====================================================================================================================


        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController.b().whileTrue(drivetrain
        //         .applyRequest(() -> point
        //                 .withModuleDirection(
        //                         new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

// ====================================================================================================================// 
// ====================================================================================================================
// ====================================================================================================================

        // // reset the field-centric heading on left bumper press
        // driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // driverController.b().onTrue(elevator.quasiStaticTest(Direction.kForward));
        // driverController.x().onTrue(elevator.dynamicTest(Direction.kReverse));
        // driverController.a().onTrue(elevator.runToPosition(2));

        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is
        // pressed,
        // cancelling on release.
        // driverController.povLeft().whileTrue(elevator.quasiStaticTest(Direction.kForward));
        // driverController.povRight().whileTrue(elevator.quasiStaticTest(Direction.kReverse));
        // driverController.povUp().whileTrue(elevator.dynamicTest(Direction.kForward));
        // driverController.povDown().whileTrue(elevator.dynamicTest(Direction.kReverse));
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

        driverController.rightBumper().onTrue(runToStore());

        driverController.leftTrigger().onTrue(coral.runMotorCommand(.7)).onFalse(coral.runMotorCommand(0));
        driverController.leftBumper().onTrue(coral.runMotorCommand(-.6)).onFalse(coral.runMotorCommand(0));

        drivetrain.registerTelemetry(logger::telemeterize);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void resetArmPid() {
        arm.pidReset();
    }

    public Command runArmMotorOff() {
        return arm.runMotorOffCommand();
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
        return elevator.runToPosition(54).alongWith(Commands.waitUntil(elevator.atTargetPosition()))
                .andThen(arm.runToPositionCommand(Units.degreesToRadians(123.)));
    }

    public Command runToL3() {
        return elevator.runToPosition(49.6).alongWith(Commands.waitUntil(elevator.atTargetPosition()))
                .andThen(arm.runToPositionCommand(Units.degreesToRadians(55)));
    }

    public Command runToL2() {
        return elevator.runToPosition(33).alongWith(Commands.waitUntil(elevator.atTargetPosition()))
                .andThen(arm.runToPositionCommand(Units.degreesToRadians(55)));
    }

    public Command runAquire() {
        return elevator.runToPosition(17.5).alongWith(Commands.waitUntil(elevator.atTargetPosition()))
                .andThen(arm.runToPositionCommand(Units.degreesToRadians(235)));
    }

    public Command runToStore() {
        return elevator.runToPosition(17.5).alongWith(Commands.waitUntil(elevator.atTargetPosition()))
                .andThen(arm.runToPositionCommand(Units.degreesToRadians(180)));
    }
}
