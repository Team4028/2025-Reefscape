// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.util.VisionUtil.LimelightSim;

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
    private final LimelightSim ll4iiiSim = new LimelightSim(ll4iii, new Transform3d());

    private static final double SLOW_SPEED = 0.1;
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
        drive.setPose(new Pose2d(drive.getPose().getTranslation(),
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? Rotation2d.kZero
                        : Rotation2d.kPi));
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
        ll4iiiSim.updateRobotPose(drive.getPose());
        Logger.recordOutput("Vision/" + ll4iiiSim.getName() + "/TagPoses", ll4iiiSim.getTagsSeen());
        RobotSim.logMechanism();
    }

    public void disableArmistice() {
        armistice.orbitalStrike();
    }

    private void configureBindings() {

        // ================= //
        /* DRIVER CONTROLLER */
        // ================= //

        // ==============================================
        // DC -- LY/LX/RX: Drive
        // ==============================================
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> scaleDriverController(() -> -driverController.getLeftY(), LimiterState.X),
                        () -> scaleDriverController(() -> -driverController.getLeftX(), LimiterState.Y),
                        () -> scaleDriverController(() -> -driverController.getRightX(),
                                LimiterState.THETA)));

        // ==============================================
        // DC -- RB: Slow
        // ==============================================
        driverController.rightBumper().onTrue(Commands.runOnce(() -> currSpeed = SLOW_SPEED))
                .onFalse(Commands.runOnce(() -> currSpeed = DEFAULT_BASE_SPEED));

        // ==============================================
        // DC -- START: Zero Drive
        // ==============================================
        driverController.start().onTrue(
                Commands.runOnce(
                        () -> drive.setPose(
                                new Pose2d(drive.getPose().getTranslation(),
                                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                                ? Rotation2d.kZero
                                                : Rotation2d.kPi)),
                        drive)
                        .ignoringDisable(true));

        // ==============================================
        // DC -- RS: Cancel Command
        // ==============================================
        driverController.rightStick().onTrue(drive.runOnce(drive::stop));

        // ==============================================
        // DC -- LT: Infeed Coral
        // ==============================================
        driverController.leftTrigger().onTrue(coral.runMotorCommand(.45)).onFalse(coral.runMotorCommand(0));

        // ==============================================
        // DC -- LB: Outfeed Coral
        // ==============================================
        driverController.leftBumper().onTrue(coral.runMotorCommand(-.40)).onFalse(coral.runMotorCommand(0));

        // ==============================================
        // OC -- START: Toggle Game Piece Mode
        // ==============================================
        operatorController.start().onTrue(Commands.runOnce(() -> armistice.setCoralMode(!armistice.getCoralMode())));

        // =================== //
        /* OPERATOR CONTROLLER */
        // =================== //

        // ==============================================
        // OC -- RB: Magic Score Right Branch
        // ==============================================
        operatorController.rightBumper().onTrue(
                Commands.runOnce(() -> drive.setReefTargetIsRight(true)).andThen(Commands.defer(this::runToClosestReef,
                        Set.<Subsystem>of(drive, armistice.getArm(), armistice.getElevator(), coral))));

        // ==============================================
        // OC -- LB: Magic Score Left Branch
        // ==============================================
        operatorController.leftBumper().onTrue(
                Commands.runOnce(() -> drive.setReefTargetIsRight(false)).andThen(Commands.defer(this::runToClosestReef,
                        Set.<Subsystem>of(drive, armistice.getArm(), armistice.getElevator(), coral))));

        // ==============================================
        // OC -- LT: Infeed Algae
        // ==============================================
        operatorController.leftTrigger().onTrue(algae.runMotorCommand(0.7)).onFalse(algae.runMotorCommand(0));

        // ==============================================
        // OC -- RT: Outfeed Algae
        // ==============================================
        operatorController.rightTrigger().onTrue(algae.runMotorCommand(-0.7)).onFalse(algae.runMotorCommand(0));

        // ==============================================
        // OC -- LY: Climber (up = climb, down = bad)
        // ==============================================
        operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.5).onTrue(climber.runVbusCommand(-0.2));
        operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value, -0.5)
                .and(operatorController.axisLessThan(XboxController.Axis.kLeftY.value, 0.5))
                .onTrue(climber.runVbusCommand(0));
        operatorController.axisLessThan(XboxController.Axis.kLeftY.value, -0.5).onTrue(climber.runVbusCommand(0.2));

        // ==============================================
        // OC -- DPAD UP: Increment Armistice Manual Index
        // ==============================================
        operatorController.povUp().onTrue(armistice.incFutureArmisticePosition());

        // ==============================================
        // OC -- DPAD DOWN: Decrement Armistice Manual Index
        // ==============================================
        operatorController.povDown().onTrue(armistice.decFutureArmisticePosition());

        // ==============================================
        // OC -- DPAD LEFT/RIGHT: Inc/Dev Magic Score Algae Height
        // ==============================================
        operatorController.povLeft().onTrue(armistice.decAutoAlgaePos());
        operatorController.povRight().onTrue(armistice.incAutoAlgaePos());

        // ==============================================
        // OC -- Y: Run To Aquire/Lollipop
        // ==============================================
        operatorController.y()
                .onTrue(Commands.defer(
                        () -> armistice.runToPositionCommand(
                                armistice.getCoralMode() ? ArmisticePositions.ACQUIRE : ArmisticePositions.LOLLIPOP),
                        Set.<Subsystem>of(armistice.getElevator(), armistice.getArm())));

        // ==============================================
        // OC -- A: Run To Manual Index Position
        // ==============================================
        operatorController.a().onTrue(armistice.runToFutureArmisticePositionCommand());

        // ==============================================
        // OC -- X: Run To Stow
        // ==============================================
        operatorController.x().onTrue(armistice.runToPositionCommand(ArmisticePositions.STOW));

        // ==============================================
        // OC -- B: Magic Score Algae
        // ==============================================
        operatorController.b().onTrue(Commands.defer(this::runToClosestAlgae,
                algaeCommandRequs().get()));

        // ==============================================
        // OC -- RX: Snap To Coral Stations
        // ==============================================
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

        // ==============================================
        // EC -- RB: Climb Climber
        // ==============================================
        emergencyController.rightBumper().onTrue(climber.runVbusCommand(0.2)).onFalse(climber.runVbusCommand(0));

        // ==============================================
        // EC -- LB: Badify Climber
        // ==============================================
        emergencyController.leftBumper().onTrue(climber.runVbusCommand(-0.2)).onFalse(climber.runVbusCommand(0));

        // ==============================================
        // EC -- DPAD UP: Nudge Elevator Up
        // ==============================================
        emergencyController.povUp().onTrue(armistice.nudgeCommand(0.5, 0));

        // ==============================================
        // EC -- DPAD DOWN: Nudge Elevator Down
        // ==============================================
        emergencyController.povDown().onTrue(armistice.nudgeCommand(-0.5, 0));

        // ==============================================
        // EC -- DPAD RIGHT: Nudge Arm CCW
        // ==============================================
        emergencyController.povRight().onTrue(armistice.nudgeCommand(0.0, 0.1));

        // ==============================================
        // EC -- DPAD LEFT: Nudge Arm CW
        // ==============================================
        emergencyController.povLeft().onTrue(armistice.nudgeCommand(0, -0.1));

        // ==============================================
        // EC -- A: Run To Climb Pos
        // ==============================================
        emergencyController.a().onTrue(armistice.runToPositionCommand(ArmisticePositions.CLIMB));
    }

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
        return AutoSequencing.autoScoreReefNoShoot(drive, armistice, coral, drive::closestReefPose,
                armistice::getFutureArmisticePositions);
    }

    private Command runToClosestAlgae() {
        return armistice.getAutoAlgaePosition() == ArmisticePositions.BARGE
                ? armistice.runToPositionCommand(ArmisticePositions.BARGE)
                : AutoSequencing.autoAquireReefAlgae(drive, armistice, algae, drive::closestReefPoseAlgae,
                        armistice::getAutoAlgaePosition);
    }

    private Supplier<Set<Subsystem>> algaeCommandRequs() {
        return () -> armistice.getAutoAlgaePosition() == ArmisticePositions.BARGE
                ? Set.<Subsystem>of(armistice.getArm(), armistice.getElevator())
                : Set.<Subsystem>of(drive, armistice.getArm(), armistice.getElevator(), algae);
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
