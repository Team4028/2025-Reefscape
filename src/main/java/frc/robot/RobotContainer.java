// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Armistice.ArmisticePositions;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MagicSequencing;
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
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({ frc.robot.util.RobotSim.class, frc.robot.commands.DriveCommands.class })
public class RobotContainer {
    public enum LimiterState {
        X,
        Y,
        THETA
    }

    private final Armistice armistice = new Armistice();
    private final CoralManipulator coral = new CoralManipulatorIOTalonFX().simSwitch();
    private final AlgaeManipulator algae = new AlgaeManipulatorIOTalonFX().simSwitch();
    private final Climber climber = new ClimberIOTalonFX().simSwitch();

    private final Limelight ll4iii = new Limelight(new LimelightIO("limelight-fouriii", true, Optional.empty()));
    private final Limelight ll4ii = new Limelight(new LimelightIO("limelight-fourii", true, Optional.empty()));

    private static final double SLOW_SPEED = 0.2;
    private static final double DEFAULT_BASE_SPEED = 0.3;
    private double currSpeed = DEFAULT_BASE_SPEED;

    private final LoggedDashboardChooser<Command> autonChooser;
    private final Drive drive = RobotSim.simSwitch(new GyroIOPigeon2(), new ModuleIO[] {
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight)
    });

    private final HumanCamera humanCam = new HumanCamera();

    @AutoLogOutput
    private boolean climbDeadmanUnsafe = false;

    private final Trigger magicAlgaeOn = new Trigger(armistice::getMagicAlgaeOn);

    @AutoLogOutput
    private final Trigger supercycleIsL4 = new Trigger(
            () -> armistice.getFutureArmisticePositions() == ArmisticePositions.Cora_L4);

    @AutoLogOutput
    private final Trigger scIsGood = new Trigger(() -> (armistice.getFutureArmisticePositions().isCoralScore()
            && armistice.getFutureArmisticePositions() != ArmisticePositions.Cora_L1)
            && (armistice.getTargetPosition() != ArmisticePositions.Cora_L4
                    || drive.driveCloseEnoughReefAuton().getAsBoolean()));

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
        NamedCommands.registerCommand("Guarentee Stop", realDrivetrainStop());
        NamedCommands.registerCommand("Acquire", coral.runMotorCommand(.7)
                .alongWith(Commands.waitUntil(
                        coral.hasGamePieceSupplier()))
                .andThen(coral.runMotorCommand(0))
                .raceWith(Commands.defer(() -> drive.translateToPositionWithPID(
                        AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(Constants.AQUIRE_POS)
                                : Constants.AQUIRE_POS),
                        Set.of(drive))));

        NamedCommands.registerCommand("Acquire Left", coral.runMotorCommand(.7)
                .alongWith(Commands.waitUntil(
                        coral.hasGamePieceSupplier()))
                .andThen(coral.runMotorCommand(0))
                .raceWith(Commands.defer(() -> drive.translateToPositionWithPID(
                        AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(Constants.AQUIRE_LEFT_POS)
                                : Constants.AQUIRE_LEFT_POS),
                        Set.of(drive))));
        NamedCommands.registerCommand("Acquire Run",
                coral.runMotorCommand(.7).alongWith(Commands.waitUntil(coral.hasGamePieceSupplier())));
        NamedCommands.registerCommand("Score Outfeed",
                Commands.waitUntil(armistice.armAndElevatorAtTarget())
                        .andThen(Commands.defer(() -> coral.runMotorCommand(getOutfeedVBus()), Set.of(coral))
                                .alongWith(Commands.waitSeconds(0.15))
                                .andThen(coral.runMotorCommand(0))));
        NamedCommands.registerCommand("WaitUntilClose", Commands.waitUntil(drive.driveCloseEnoughReefAuton()));
        NamedCommands.registerCommand("WaitUntilCloseAcq", Commands.waitUntil(drive.driveCloseEnoughAcquireAuton()));
        NamedCommands.registerCommand("Run To Closest Right Reef",
                rightPidToClosestReefAuton().until(drive.translatePidInPosition()).withTimeout(1));
        NamedCommands.registerCommand("Run To Closest Left Reef",
                leftPidToClosestReefAuton().until(drive.translatePidInPosition()).withTimeout(1));

        NamedCommands.registerCommand("L4 Score",
                runToPositionDeferredClosestReefJSONOffset(() -> ArmisticePositions.Cora_L4));
        NamedCommands.registerCommand("Stow", armistice.runToPositionCommand(ArmisticePositions.STOW));
        NamedCommands.registerCommand("Stow No Wait", armistice.runToPositionNoWait(ArmisticePositions.STOW));
        NamedCommands.registerCommand("Acquire Pos",
                runToPositionDeferredClosestReefJSONOffset(() -> ArmisticePositions.CLEAN));
        NamedCommands.registerCommand("L3 Score",
                runToPositionDeferredClosestReefJSONOffset(() -> ArmisticePositions.Cora_L3));
        NamedCommands.registerCommand("L2 Score",
                runToPositionDeferredClosestReefJSONOffset(() -> ArmisticePositions.Cora_L2));
        NamedCommands.registerCommand("Blip",
                coral.runMotorCommand(.7).alongWith(Commands.waitSeconds(0.25)).andThen(coral.runMotorCommand(0)));
        NamedCommands.registerCommand("SuperCycle L4", Commands.defer(
                () -> MagicSequencing.magicScoreSuperCycleL4(drive, armistice, coral, algae,
                        drive::closestReefPose,
                        drive::closestReefPoseAlgae,
                        () -> ArmisticePositions.Cora_L4,
                        armistice::getAutoAlgaePosition),
                Set.of(drive, armistice.getArm(), armistice.getElevator(), coral, algae)));
        autonChooser = new LoggedDashboardChooser<>("Auton Chooser", AutoBuilder.buildAutoChooser());
        autonChooser.addOption("Char drivetrain", drive.feedforwardCharacterization());
        autonChooser.addOption("Char Wheel Radius", drive.wheelRadiusCharacterization());
        // Set up SysId routines
        VisionUtil.bindSimCameras(new Transform3d[] { new Transform3d() });
        configureBindings();
    }

    public void updateArmisticeAutoAlgae() {
        armistice.updateAutoAlgaePos(drive.closestReefTag());
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
        VisionUtil.setLLIMUModes(on);
    }

    public void turnOnIfGood() {
        if (!VisionUtil.requestingSeed
                || VisionUtil.poseSources.keySet().stream()
                        .allMatch(ll -> Math.abs(drive.getRotation()
                                .minus(Rotation2d.fromDegrees(ll.getGoodActualAngleToFixProbelmsOrbitalStrikeV2()))
                                .getDegrees()) < 0.02)) {
            VisionUtil.setLLIMUModes(true);
            VisionUtil.requestingSeed = false;
        } else {
            VisionUtil.setLLIMUModes(false);
            VisionUtil.requestingSeed = true;
        }
    }

    public void logLLPoses() {
        VisionUtil.logPoses(drive);
    }

    public void seedll4IMU() {
        VisionUtil.seedIMUs(drive.getPose().getRotation().getDegrees());
    }

    public final void simCallback() {
        RobotSim.update(armistice.getSimData());
        VisionUtil.updateSimDrivePose(drive.getPose());
        VisionUtil.logSeenTags();
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
                drive.joystickDrive(
                        () -> scaleDriverController(() -> -driverController.getLeftY(), LimiterState.X),
                        () -> scaleDriverController(() -> -driverController.getLeftX(), LimiterState.Y),
                        () -> scaleDriverController(() -> -driverController.getRightX(),
                                LimiterState.THETA)));

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
        driverController.x().onTrue(drive.runOnce(drive::stopWithX));

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
        driverController.leftBumper()
                .onTrue(Commands.defer(() -> coral.runMotorCommand(getOutfeedVBus()), Set.of(coral)))
                .onFalse(coral.runMotorCommand(0));

        driverController.rightBumper().onTrue(Commands.runOnce(() -> currSpeed = SLOW_SPEED))
                .onFalse(Commands.runOnce(() -> currSpeed = DEFAULT_BASE_SPEED));

        // ==============================================
        // OC -- START: Toggle Game Piece Mode
        // ==============================================
        operatorController.start().onTrue(armistice.toggleCoralMode());

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

        operatorController.back().onTrue(armistice.toggleAutoAlgae());

        // ==============================================
        // OC -- LT: Infeed Algae
        // ==============================================
        operatorController.leftTrigger().onTrue(algae.runMotorCommand(0.7)).onFalse(algae.runMotorCommand(0));

        // ==============================================
        // OC -- RT: Outfeed Algae
        // ==============================================
        operatorController.rightTrigger()
                .onTrue(Commands.defer(() -> algae.runMotorCommand(getAlgaeOutfeedVBus()), Set.of(algae)))
                .onFalse(algae.runMotorCommand(0));

        // ==============================================
        // OC -- LY: Climber (up = climb, down = bad)
        // ==============================================
        // operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value,
        // 0.5).onTrue(climber.runVbusCommand(-0.4));
        operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value, -0.5)
                .onTrue(climber.runVbusCommand(0));
        operatorController.axisLessThan(XboxController.Axis.kLeftY.value, -0.5)
                .onTrue(climber.runVbusCommand(0.7).onlyIf(() -> climbDeadmanUnsafe));

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
        operatorController.povLeft().onTrue(armistice.decFutureAquirePos());
        operatorController.povRight().onTrue(armistice.incFutureAquirePos());

        // ==============================================
        // OC -- Y: Run To Aquire/Lollipop
        // ==============================================
        operatorController.y()
                .onTrue(armistice.runToFutureAquirePositionCommand(drive::closestReefName,
                        drive::getReefTargetIsRight).onlyIf(() -> !algae.hasGamePieceSupplier().getAsBoolean()));

        // ==============================================
        // OC -- A: Run To Manual Index Position
        // ==============================================
        operatorController.a().onTrue(
                armistice.runToFutureArmisticePositionCommand(drive::closestReefName, drive::getReefTargetIsRight));

        // ==============================================
        // OC -- X: Run To Stow
        // ==============================================
        operatorController.x().onTrue(armistice.runToPositionCommand(ArmisticePositions.STOW));

        // ==============================================
        // OC -- B: Magic Score Algae
        // ==============================================
        // operatorController.b()
        // .onTrue(runToClosestAlgae().andThen(
        // Commands.runOnce(() ->
        // armistice.setFutureArmisticePosition(ArmisticePositions.Cora_L3))
        // .onlyIf(() -> armistice.getAutoAlgaePosition() !=
        // ArmisticePositions.BARGE)));

        // operatorController.b().and(magicAlgaeOn).onTrue(runToClosestAlgae())
        // .onTrue(armistice.setFutureArmisticePosition(ArmisticePositions.Cora_L3));
        // operatorController.b().and(magicAlgaeOn).onTrue(Commands.defer(this::runToClosestSuperCycle,
        // Set.of(drive, armistice.getArm(), armistice.getElevator(), coral, algae)));
        operatorController.b().and(magicAlgaeOn).and(supercycleIsL4).and(scIsGood).onTrue(Commands
                .defer(this::runMagicBackupAlgaeL4, Set.of(drive, armistice.getArm(), armistice.getElevator(), algae)));

        operatorController.b().and(magicAlgaeOn).and(scIsGood).and(supercycleIsL4.negate()).onTrue(Commands.defer(
                this::runMagicAlgaeLOther, Set.of(drive, armistice.getArm(), armistice.getElevator(), algae, coral)));

        operatorController.b().and(magicAlgaeOn.negate())
                .onTrue(armistice.runToPositionCommand(ArmisticePositions.BARGE));

        // ==============================================
        // OC -- RX: Snap To Coral Stations
        // ==============================================
        operatorController.axisGreaterThan(XboxController.Axis.kRightX.value, 0.5)
                .onTrue(drive.joystickDriveAtAngle(
                        () -> scaleDriverController(() -> -driverController.getLeftY(), LimiterState.X),
                        () -> scaleDriverController(() -> -driverController.getLeftX(), LimiterState.Y),
                        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                                ? Rotation2d.fromDegrees(Constants.CORAL_STATION_RIGHT_ROTATION_DEG)
                                : FlippingUtil.flipFieldRotation(
                                        Rotation2d.fromDegrees(Constants.CORAL_STATION_RIGHT_ROTATION_DEG))));
        operatorController.axisLessThan(XboxController.Axis.kRightX.value, -0.5)
                .onTrue(drive.joystickDriveAtAngle(
                        () -> scaleDriverController(() -> -driverController.getLeftY(), LimiterState.X),
                        () -> scaleDriverController(() -> -driverController.getLeftX(), LimiterState.Y),
                        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                                ? Rotation2d.fromDegrees(Constants.CORAL_STATION_LEFT_ROTATION_DEG)
                                : FlippingUtil.flipFieldRotation(
                                        Rotation2d.fromDegrees(Constants.CORAL_STATION_LEFT_ROTATION_DEG))));

        // ==================== //
        /* EMERGENCY CONTROLLER */
        // ==================== //

        // ==============================================
        // EC -- DPAD: Global Nudges
        // ==============================================
        emergencyController.povUp().onTrue(armistice.nudgeCommandGlobalPermanant(1,
                0));
        emergencyController.povDown().onTrue(armistice.nudgeCommandGlobalPermanant(-1,
                0));
        emergencyController.povRight().onTrue(armistice.nudgeCommandPermanant(0,
                Units.degreesToRadians(1)));
        emergencyController.povLeft().onTrue(armistice.nudgeCommandPermanant(0,
                Units.degreesToRadians(-1)));

        // ==============================================
        // EC -- DPAD: Positional Nudges
        // ==============================================
        emergencyController.y().onTrue(armistice.nudgeCommandPermanant(1, 0));
        emergencyController.a().onTrue(armistice.nudgeCommandPermanant(-1, 0));
        emergencyController.b().onTrue(armistice.nudgeCommandPermanant(0,
                Units.degreesToRadians(1)));
        emergencyController.x().onTrue(armistice.nudgeCommandPermanant(0,
                Units.degreesToRadians(-1)));

        // ==============================================
        // EC -- START: Run To Climb Position
        // ==============================================
        emergencyController.start().onTrue(armistice.runToPositionCommand(ArmisticePositions.CLIMB));

        emergencyController.back().onTrue(armistice.runToPositionCommand(ArmisticePositions.CLIMB_2));

        emergencyController.rightStick().onTrue(armistice.resetNudges().ignoringDisable(true));

        emergencyController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.4)
                .onTrue(armistice.runToPositionCommand(ArmisticePositions.LOLI));

        emergencyController.axisMagnitudeGreaterThan(XboxController.Axis.kRightX.value, 0.5)
                .onTrue(armistice.toggleCoralReefOffset());

        emergencyController.axisMagnitudeGreaterThan(XboxController.Axis.kLeftX.value, 0.5)
                .onTrue(Commands.runOnce(() -> drive.setReefTargetIsRight(
                        Math.signum(emergencyController.getRawAxis(XboxController.Axis.kLeftX.value)) > 0))
                        .ignoringDisable(true));

        emergencyController.rightBumper()
                .onTrue(Commands.runOnce(() -> humanCam.setCamera(climbDeadmanUnsafe = !climbDeadmanUnsafe))
                        .ignoringDisable(true));
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

    private Command runToPositionDeferredClosestReefJSONOffset(Supplier<ArmisticePositions> position) {
        return Commands.defer(
                () -> armistice.runToPositionCommand(position.get(),
                        drive.closestReefName(), drive.getReefTargetIsRight()),
                Set.of(armistice.getArm(), armistice.getElevator()));
    }

    private Command runToClosestReef() {
        return armistice.magicIsSnap() ? magicSnapL1()
                : MagicSequencing.magicScoreNoScoreReefOnlyPID(drive, armistice, coral,
                        () -> armistice.getFutureArmisticePositions().isPipe() ? drive.pipe1ClosestReefPose()
                                : drive.closestReefPose(),
                        armistice::getFutureArmisticePositions);
    }

    // private Command runToClosestSuperCycle() {
    // if (!armistice.getFutureArmisticePositions().isCoralScore()
    // || armistice.getFutureArmisticePositions() == ArmisticePositions.Cora_L1)
    // return Commands.none();
    // else if (armistice.getTargetPosition() == ArmisticePositions.Cora_L4)
    // return drive.driveCloseEnoughReefAuton().getAsBoolean()
    // ? MagicSequencing.magicBackUpAndMagicAlgaeL4(drive, armistice, algae,
    // drive::closestReefPose,
    // drive::closestReefPoseAlgae,
    // armistice::getAutoAlgaePosition).withName("MAGICALGAEL4")
    // : Commands.none();
    // else
    // return MagicSequencing
    // .magicScoreSuperCycleLOther(drive, armistice, coral, algae,
    // drive::closestReefPose,
    // drive::closestReefPoseAlgae,
    // armistice::getFutureArmisticePositions, armistice::getAutoAlgaePosition)
    // .withName("MAGICALGAELOTHER");
    // }
    private Command runMagicBackupAlgaeL4() {
        return MagicSequencing.magicBackUpAndMagicAlgaeL4(drive, armistice, algae, drive::closestReefPose,
                drive::closestReefPoseAlgae, armistice::getAutoAlgaePosition);
    }

    private Command runMagicAlgaeLOther() {
        return MagicSequencing.magicScoreSuperCycleLOther(drive, armistice, coral, algae, drive::closestReefPose,
                drive::closestReefPoseAlgae, armistice::getFutureArmisticePositions, armistice::getAutoAlgaePosition);
    }

    private double getAlgaeOutfeedVBus() {
        return armistice.getTargetPosition() == ArmisticePositions.LOLI ? -.5 : -.8;
    }

    private Command magicSnapL1() {
        return armistice.runToPositionNoWait(ArmisticePositions.Cora_L1).alongWith(drive.joystickDriveAtAngle(
                () -> scaleDriverController(() -> -driverController.getLeftY(), LimiterState.X),
                () -> scaleDriverController(() -> -driverController.getLeftX(), LimiterState.Y),
                drive::closestReefL1Rotation));
    }

    private Command rightPidToClosestReefAuton() {
        return Commands.runOnce(() -> drive.setReefTargetIsRight(true)).andThen(Commands
                .defer(() -> drive.translateToPositionWithPID(drive.closestReefPose()), Set.<Subsystem>of(drive)));
    }

    private Command leftPidToClosestReefAuton() {
        return Commands.runOnce(() -> drive.setReefTargetIsRight(false)).andThen(Commands
                .defer(() -> drive.translateToPositionWithPID(drive.closestReefPose()), Set.<Subsystem>of(drive)));
    }

    private Command runToClosestAlgae() {
        return Commands.defer(
                () -> MagicSequencing.magicGetAlgaeOnlyPID(drive, armistice, algae, drive::closestReefPoseAlgae,
                        armistice::getAutoAlgaePosition),
                Set.of(drive, armistice.getArm(), armistice.getElevator(), algae));
    }

    private double scaleDriverController(DoubleSupplier controllerInput, LimiterState type) {
        double input = controllerInput.getAsDouble() * ((currSpeed)
                + (currSpeed == SLOW_SPEED ? 0 : driverController.getRightTriggerAxis() * (1 - currSpeed)));
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

    private double getOutfeedVBus() {
        return armistice.getElevatorPosition() > 45 ? -.8
                : armistice.getTargetPosition() == ArmisticePositions.Cora_L1 ? -.8 : -.4;
    }

    public Command realDrivetrainStop() {
        return drive.runOnce(drive::stop);
    }

}
