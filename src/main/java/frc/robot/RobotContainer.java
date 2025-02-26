// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Armistice.ArmisticePositions;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algae.AlgaeManipulator;
import frc.robot.subsystems.algae.AlgaeManipulatorIOTalonSRX;
import frc.robot.subsystems.coral.CoralManipulator;
import frc.robot.subsystems.coral.CoralManipulatorIOTalonSRX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.limelight.LimelightConstants;
import frc.robot.subsystems.limelight.LimelightIO;
import frc.robot.subsystems.limelight.LimelightIO.LoggablePoseEstimate;
import frc.robot.util.RobotSim;
import frc.robot.util.VisionUtil;

public class RobotContainer {
    private final CoralManipulator coral = RobotSim
            .coralManipulatorSimSwitch(new CoralManipulatorIOTalonSRX());

    private final Drive drive = RobotSim.driveSimSwitch(new GyroIOPigeon2(),
            new ModuleIO[] { new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                    new ModuleIOTalonFX(TunerConstants.BackRight) });

    private final AlgaeManipulator algae = RobotSim.algaeSimSwitch(new AlgaeManipulatorIOTalonSRX());
    private final Armistice armistice = new Armistice();
    private final Limelight ll4;
    private final Limelight ll4station;

    private final SlewRateLimiter xLimiter, yLimiter, thetaLimiter;
    private static final double DEFAULT_BASE_SPEED = 0.2;

    private final LoggedDashboardChooser<Command> autoChooser;

    private final CommandXboxController driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(
            OperatorConstants.kOperatorControllerPort);

    public RobotContainer() {
        ll4 = new Limelight(new LimelightIO("limelight-fourii", true, Optional.empty()));
        ll4station = new Limelight(new LimelightIO("limelight-fouriii", true, Optional.empty()));
        xLimiter = new SlewRateLimiter(4);
        yLimiter = new SlewRateLimiter(4);
        thetaLimiter = new SlewRateLimiter(4);
        NamedCommands.registerCommand("Guarentee Stop", realDrivetrainStop());
        NamedCommands.registerCommand("Acquire", coral.runMotorCommand(.7)
                .alongWith(Commands.waitUntil(
                        coral.hasGamePieceSupplier()))
                .andThen(coral.runMotorCommand(0)));
        NamedCommands.registerCommand("Score Outfeed",
                Commands.waitUntil(armistice.armAndElevatorAtTarget()).andThen(Commands.waitSeconds(0.5))
                        .andThen(coral.runMotorCommand(-.8).alongWith(Commands.waitSeconds(1))
                                .andThen(coral.runMotorCommand(0))));

        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                DriveCommands.wheelRadiusCharacterization(drive));
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

        autoChooser.addOption("Do nothing", Commands.none());
        configureBindings();
    }

    private void addVisionMeasurement(LoggablePoseEstimate poseEstimate) {
        drive.addVisionMeasurement(poseEstimate.pose(), poseEstimate.timestampSeconds(),
                LimelightConstants.GOOD_STD_DEVS);
    }

    public void addMeasurements() {
        VisionUtil.addMeasurements(this::addVisionMeasurement, drive);
    }

    public Command addMeasurementsCommand() {
        return VisionUtil.addMeasurementsCommand(this::addVisionMeasurement, drive);
    }

    public void subsystemWarmup() {
        ll4.warmup(); // just you for now
        ll4station.warmup();
        logLLPoses();
    }

    public final void simCallback() {
        RobotSim.update(armistice.getSimData());

        RobotSim.logMechanism();
    }

    public void periodicLL4IMU(boolean on) {
        ll4.setIMUInternal(on);
        ll4station.setIMUInternal(on);
    }

    public void logLLPoses() {
        VisionUtil.logPoses(drive);
    }

    public void seedll4IMU() {
        ll4.seedLLSolverYaw(drive.getPose().getRotation().getDegrees());
        ll4station.seedLLSolverYaw(drive.getPose().getRotation().getDegrees());
    }

    public void disableArmistice() {
        armistice.orbitalStrike();
    }

    private void configureBindings() {
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> scaleDriverController(() -> -driverController.getLeftY(), xLimiter),
                        () -> scaleDriverController(() -> -driverController.getLeftX(), yLimiter),
                        () -> scaleDriverController(() -> -driverController.getRightX(),
                                thetaLimiter)));
        //Coral Manip
        driverController.leftTrigger().onTrue(coral.runMotorCommand(.7)
        .alongWith(Commands.waitUntil(
                coral.hasGamePieceSupplier()))
        .andThen(coral.runMotorCommand(0)));

        driverController.leftBumper().onTrue(coral.runMotorCommand(-.8)).onFalse(coral.runMotorCommand(0));

        //Nudge commands
        //elevator 
        driverController.povUp().onTrue(armistice.nudgeCommand(.5, 0));
        driverController.povDown().onTrue(armistice.nudgeCommand(-.5, 0));
        //pivot nudge
        driverController.povLeft().onTrue(armistice.nudgeCommand(0, .1));
        driverController.povRight().onTrue(armistice.nudgeCommand(0, -.1));

        //Algae Manip
        operatorController.leftTrigger().onTrue(algae.runMotorCommand(.7)).onFalse(algae.runMotorCommand(0));
        operatorController.rightTrigger().onTrue(algae.runMotorCommand(-.7)).onFalse(algae.runMotorCommand(0));
        //Elevator
        operatorController.rightBumper().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.STOW));
        operatorController.y().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.ACQUIRE));
        operatorController.x().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.L4));
        operatorController.b().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.L3));
        operatorController.a().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.L2));
        operatorController.povUp().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.ALGAE_AQUIRE_L2));
        operatorController.povDown().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.ALGAE_AQUIRE_L3));

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
                .runOnce(drive::stop);
    }

    private double scaleDriverController(DoubleSupplier controllerInput, SlewRateLimiter limiter) {
        return limiter.calculate(
                controllerInput.getAsDouble() * (DEFAULT_BASE_SPEED
                        + (driverController.getRightTriggerAxis() * (1 - DEFAULT_BASE_SPEED))));
    }
}
