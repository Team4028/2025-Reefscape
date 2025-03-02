// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Armistice.ArmisticePositions;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoSequencing;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algae.AlgaeManipulator;
import frc.robot.subsystems.algae.AlgaeManipulatorIOTalonFX;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.coral.CoralManipulator;
import frc.robot.subsystems.coral.CoralManipulatorIOTalonFX;
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
    public enum LimiterState {
        X,
        Y,
        THETA
    }

    private final Armistice armistice = new Armistice();
    private final CoralManipulator coral = new CoralManipulator(new CoralManipulatorIOTalonFX());
    private final AlgaeManipulator algae = new AlgaeManipulator(new AlgaeManipulatorIOTalonFX());
    private final Climber climber = new Climber(new ClimberIOTalonFX());
    // private final Limelight ll4ii = new Limelight(new
    // LimelightIO("limelight-fourii", true, null));
    private final Limelight ll4iii = new Limelight(new LimelightIO("limelight-fouriii", true, Optional.empty()));

    private static final double SLOW_SPEED = 0.07;
    private static final double DEFAULT_BASE_SPEED = 0.3;
    private double currSpeed = DEFAULT_BASE_SPEED;

    private final LoggedDashboardChooser<Command> autonChooser = new LoggedDashboardChooser<>("Auton Chooser");
    private final Drive drive = RobotSim.driveSimSwitch(new GyroIOPigeon2(), new ModuleIO[] {
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight)
    });

    // add actual limits
    private final SlewRateLimiter xLimiterL4, yLimiterL4, thetaLimiterL4, xLimiter, yLimiter, thetaLimiter;

    private final CommandXboxController driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(
            OperatorConstants.kOperatorControllerPort);
    private final CommandXboxController emergencyController = new CommandXboxController(
            OperatorConstants.kEmergencyControllerPort);

    public RobotContainer() {
        xLimiterL4 = new SlewRateLimiter(1.0);
        yLimiterL4 = new SlewRateLimiter(1.0);
        thetaLimiterL4 = new SlewRateLimiter(1.0);

        xLimiter = new SlewRateLimiter(4.0);
        yLimiter = new SlewRateLimiter(4.0);
        thetaLimiter = new SlewRateLimiter(4.0);

        autonChooser.addDefaultOption("Char drivetrain", DriveCommands.feedforwardCharacterization(drive));
        // Set up SysId routines
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

    public void periodicLL4IMU(boolean on) {
        ll4iii.setIMUInternal(on);
    }

    public void logLLPoses() {
        VisionUtil.logPoses(drive);
    }

    public void seedll4IMU() {
        ll4iii.seedLLSolverYaw(drive.getPose().getRotation().getDegrees());
    }

    public final void simCallback() {
        RobotSim.update(armistice.getSimData());

        RobotSim.logMechanism();
    }

    public void disableArmistice() {
        armistice.orbitalStrike();
    }

    private void configureBindings() {
        driverController.rightBumper().onTrue(Commands.runOnce(() -> currSpeed = SLOW_SPEED))
                .onFalse(Commands.runOnce(() -> currSpeed = DEFAULT_BASE_SPEED));

        // Reset gyro to 0° when start button is pressed
        driverController.start().onTrue(
                Commands.runOnce(
                        () -> drive.setPose(
                                new Pose2d(drive.getPose().getTranslation(),
                                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                                ? Rotation2d.kZero
                                                : Rotation2d.kPi)),
                        drive)
                        .ignoringDisable(true));

        driverController.rightStick().onTrue(drive.runOnce(drive::stop));

        driverController.leftTrigger().onTrue(coral.runMotorCommand(.45)).onFalse(coral.runMotorCommand(0));
        driverController.leftBumper().onTrue(coral.runMotorCommand(-.40)).onFalse(coral.runMotorCommand(0));
        // operator
        operatorController.start().onTrue(Commands.runOnce(() -> armistice.setCoralMode(!armistice.getCoralMode())));
        operatorController.rightBumper().onTrue(
                Commands.runOnce(() -> drive.setReefTargetIsRight(true)).andThen(Commands.defer(this::runToClosestReef,
                        Set.<Subsystem>of(drive, armistice.getArm(), armistice.getElevator(), coral))));
        operatorController.leftBumper().onTrue(
                Commands.runOnce(() -> drive.setReefTargetIsRight(false)).andThen(Commands.defer(this::runToClosestReef,
                        Set.<Subsystem>of(drive, armistice.getArm(), armistice.getElevator(), coral))));
        operatorController.leftTrigger().onTrue(algae.runMotorCommand(0.7)).onFalse(algae.runMotorCommand(0));
        operatorController.leftBumper().onTrue(algae.runMotorCommand(-0.7)).onFalse(algae.runMotorCommand(0));
        operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.5).onTrue(climber.runVbusCommand(-0.2));
        operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value, -0.5)
                .and(operatorController.axisLessThan(XboxController.Axis.kLeftY.value, 0.5))
                .onTrue(climber.runVbusCommand(0));
        operatorController.axisLessThan(XboxController.Axis.kLeftY.value, -0.5).onTrue(climber.runVbusCommand(0.2));
        operatorController.povUp().onTrue(armistice.incFutureArmisticePosition());
        operatorController.povDown().onTrue(armistice.decFutureArmisticePosition());
        operatorController.y().onTrue(armistice.runToPositionCommand(ArmisticePositions.ACQUIRE));
        operatorController.a().onTrue(armistice.runToFutureArmisticePositionCommand());
        operatorController.x().onTrue(armistice.runToPositionCommand(ArmisticePositions.STOW));
        operatorController.axisGreaterThan(XboxController.Axis.kRightX.value, 0.5)
                .onTrue(DriveCommands.joystickDriveAtAngle(drive,
                        () -> scaleDriverController(() -> -driverController.getLeftY(), LimiterState.X),
                        () -> scaleDriverController(() -> -driverController.getLeftX(), LimiterState.Y),
                        () -> Rotation2d.fromDegrees(Constants.CORAL_STATION_RIGHT_ROTATION_DEG)));
        operatorController.axisLessThan(XboxController.Axis.kRightX.value, -0.5)
                .onTrue(DriveCommands.joystickDriveAtAngle(drive,
                        () -> scaleDriverController(() -> -driverController.getLeftY(), LimiterState.X),
                        () -> scaleDriverController(() -> -driverController.getLeftX(), LimiterState.Y),
                        () -> Rotation2d.fromDegrees(Constants.CORAL_STATION_LEFT_ROTATION_DEG)));
        // ==================== //
        /* EMERGENCY CONTROLLER */
        // ==================== //
        emergencyController.rightBumper().onTrue(climber.runVbusCommand(0.2)).onFalse(climber.runVbusCommand(0));
        emergencyController.leftBumper().onTrue(climber.runVbusCommand(-0.2)).onFalse(climber.runVbusCommand(0));

        emergencyController.povUp().onTrue(armistice.nudgeCommand(0.5, 0));
        emergencyController.povDown().onTrue(armistice.nudgeCommand(-0.5, 0));
        emergencyController.povRight().onTrue(armistice.nudgeCommand(0.0, 0.1));
        emergencyController.povLeft().onTrue(armistice.nudgeCommand(0, -0.1));
        emergencyController.a().onTrue(armistice.runToPositionCommand(ArmisticePositions.CLIMB));

        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> scaleDriverController(() -> -driverController.getLeftY(), LimiterState.X),
                        () -> scaleDriverController(() -> -driverController.getLeftX(), LimiterState.Y),
                        () -> scaleDriverController(() -> -driverController.getRightX(),
                                LimiterState.THETA)));
    }

    // public void resetArmPid() {
    // armistice.resetArmPid();
    // }

    public Command getAutonomousCommand() {
        return autonChooser.get();
    }

    public double chooseXLimiter(double input) {
        var a = armistice.getElevatorPosition();
        if (a > 40) {
            return xLimiterL4.calculate(input);
        } else {
            return xLimiter.calculate(input);
        }

    }

    public double chooseYLimiter(double input) {
        var a = armistice.getElevatorPosition();
        if (a > 40) {
            return yLimiterL4.calculate(input);
        } else {
            return yLimiter.calculate(input);
        }
    }

    public double chooseThetaLimiter(double input) {
        var a = armistice.getElevatorPosition();
        if (a > 40) {
            return thetaLimiterL4.calculate(input);
        } else {
            return thetaLimiter.calculate(input);
        }
    }

    private Command runToClosestReef() {
        return AutoSequencing.autoScoreReef(drive, armistice, coral, drive::closestReefPose,
                armistice::getFutureArmisticePositions);
    }

    private double scaleDriverController(DoubleSupplier controllerInput, LimiterState type) {
        double input = controllerInput.getAsDouble() * ((currSpeed)
                + (driverController.getRightTriggerAxis() * (1 - currSpeed)));
        switch (type) {
            case X:
                return chooseXLimiter(input);
            case Y:
                return chooseYLimiter(input);
            case THETA:
                return chooseThetaLimiter(input);
            default:
                return 0.0;
        }
    }

}
