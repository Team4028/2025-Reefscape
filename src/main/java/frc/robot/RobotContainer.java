// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Armistice.ArmisticePositions;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.coral.CoralManipulator;
import frc.robot.subsystems.coral.CoralManipulatorIOTalonSRX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.RobotSim;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
    private final CoralManipulator coralManipulator = new CoralManipulator(
            RobotSim.coralManipulatorSimSwitch(new CoralManipulatorIOTalonSRX()));

    private final Drive drive = RobotSim.driveSimSwitch(new GyroIOPigeon2(),
            new ModuleIO[] { new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight), new ModuleIOTalonFX(TunerConstants.BackLeft),
                    new ModuleIOTalonFX(TunerConstants.BackRight) });

    private final Armistice armistice = new Armistice();

    private final SlewRateLimiter xLimiter, yLimiter, thetaLimiter;
    private static final double DEFAULT_BASE_SPEED = 0.6;

    private final LoggedDashboardChooser<Command> autoChooser;

    private final CommandXboxController driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        xLimiter = new SlewRateLimiter(4);
        yLimiter = new SlewRateLimiter(4);
        thetaLimiter = new SlewRateLimiter(4);
        NamedCommands.registerCommand("Guarentee Stop", realDrivetrainStop());
        NamedCommands.registerCommand("Acquire",
                armistice.runToPositionCommand(() -> ArmisticePositions.ACQUIRE)
                        .andThen(coralManipulator.runMotorCommand(.7)
                                .alongWith(Commands.waitUntil(
                                        coralManipulator.hasGamePieceSupplier()))
                                .andThen(coralManipulator.runMotorCommand(0))));
        NamedCommands.registerCommand("Score Outfeed",
                Commands.waitUntil(armistice.armAndElevatorAtTarget()).andThen(Commands.waitSeconds(0.5))
                        .andThen(coralManipulator.runMotorCommand(-.8).alongWith(Commands.waitSeconds(1))
                                .andThen(coralManipulator.runMotorCommand(0))));

        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        configureBindings();
    }

    public final void simCallback() {
        RobotSim.update(armistice.getSimData());

        RobotSim.logMechanism();
    }

    private void configureBindings() {
        drive.setDefaultCommand(
        DriveCommands.joystickDrive(
        drive,
        () -> scaleDriverController(() -> -driverController.getLeftY(), xLimiter),
        () -> scaleDriverController(() -> -driverController.getLeftX(), yLimiter),
        () -> scaleDriverController(() -> -driverController.getRightX(),
        thetaLimiter)));

        // Run to L4
        driverController.x().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.L4));
        // Run to L3
        driverController.a().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.L3));
        // Run to L2
        driverController.b().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.L2));
        // Acquire
        driverController.y().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.ACQUIRE));
        // Stow
        driverController.rightBumper().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.STOW));

        // //Nudges
        driverController.povUp().onTrue(armistice.nudgeCommand(1, 0));
        driverController.povDown().onTrue(armistice.nudgeCommand(-1, 0));
        driverController.povLeft().onTrue(armistice.nudgeCommand(0, 1));
        driverController.povRight().onTrue(armistice.nudgeCommand(0, -1));

        // Reset gyro to 0° when start button is pressed
        driverController.start().onTrue(
                Commands.runOnce(
                        () -> drive.setPose(
                                new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                        drive)
                        .ignoringDisable(true));
    }

    public void resetArmPid() {
        armistice.resetArmPid();
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public Command realDrivetrainStop() {
        return drive
                .runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0)));
    }

    private double scaleDriverController(DoubleSupplier controllerInput, SlewRateLimiter limiter) {
        return limiter.calculate(
                controllerInput.getAsDouble() * (DEFAULT_BASE_SPEED
                        + (driverController.getRightTriggerAxis() * (1 - DEFAULT_BASE_SPEED))));
    }
}
