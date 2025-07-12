// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Armistice.ArmisticePositions;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MagicSequencing;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.cage.Cage;
import frc.robot.subsystems.cage.CageIOTalonFX;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.groundinfeed.Grond;
import frc.robot.subsystems.groundinfeed.GrondIOTalonFX;
import frc.robot.subsystems.groundinfeed.GrondTOFIOPWF;
import frc.robot.subsystems.infeedpivot.InfeedPivot;
import frc.robot.subsystems.infeedpivot.InfeedPivotConstants.InfeedPivotPositions;
import frc.robot.subsystems.infeedpivot.InfeedPivotEncoderIOCancoder;
import frc.robot.subsystems.infeedpivot.InfeedPivotMotorIOTalonFXCCSource;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.limelight.LimelightConstants;
import frc.robot.subsystems.limelight.LimelightIO;
import frc.robot.subsystems.limelight.LimelightIO.LoggablePoseEstimate;
import frc.robot.subsystems.stick.WhipStick;
import frc.robot.subsystems.stick.WhipStickIOTalonFX;
import frc.robot.util.LoggedTunables.LoggedChangableBoolean;
import frc.robot.util.LoggedTunables.LoggedTunableNumber;
import frc.robot.util.MiscUtils;
import frc.robot.util.RobotSim;
import frc.robot.util.VisionUtil;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@ExtensionMethod({frc.robot.util.RobotSim.class, frc.robot.commands.DriveCommands.class, MiscUtils.class})
public class RobotContainer {
    private static final double SLOW_SPEED = 0.2;
    private static final double DEFAULT_BASE_SPEED = 0.3;
    private static final double ABORT_THRESHOLD_SEC = 12;
    private final WhipStick coral = new WhipStickIOTalonFX().simSwitch();
    private final Armistice armistice = new Armistice(coral.hasAlgae());
    private final Climber climber = new ClimberIOTalonFX().simSwitch();
    private final InfeedPivot pivot = new InfeedPivot(new InfeedPivotMotorIOTalonFXCCSource(),
            new InfeedPivotEncoderIOCancoder());
    private final Cage cage = new Cage(new CageIOTalonFX());
    private final Grond infeed = new Grond(new GrondIOTalonFX(true), new GrondIOTalonFX(false), new GrondTOFIOPWF(),
            pivot.isDownPositional());
    private boolean hasDeployed = false;
    private final Trigger hasGamePiece = new Trigger(
            infeed.hasGamepieceSupplier().bsand(() -> armistice.getTargetPosition() == ArmisticePositions.CLEAN)
                    .bsand(pivot.isDownPositional()))
            .debounce(0.080);
    private final Trigger hasGPRaw = new Trigger(infeed.hasGamepieceSupplierRawTOF());
    @AutoLogOutput
    private final Trigger needsUnjam = new Trigger(infeed.isJammed());
    private final Timer abortGrondLastScore = new Timer();
    private final Limelight ll4iii = new Limelight(new LimelightIO("limelight-fouriii", true, Optional.empty(), true));
    private final Limelight ll4ii = new Limelight(new LimelightIO("limelight-fourii", true, Optional.empty(), true));
    private final Limelight ll3Coral = new Limelight(
            new LimelightIO("limelight-threei", false, Optional.empty(),
                    new Transform3d(
                            new Translation3d(Units.inchesToMeters(-9), Units.inchesToMeters(-10),
                                    Units.inchesToMeters(21.75)),
                            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(270))),
                    false));
    @AutoLogOutput
    private final boolean isRRelative = false;
    private final LoggedDashboardChooser<Command> autonChooser;
    private final Drive drive = RobotSim.simSwitch(new GyroIOPigeon2(), new ModuleIO[]{
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight)
    });
    private final HumanCamera humanCam = new HumanCamera();
    private final LoggedChangableBoolean zeroArm = new LoggedChangableBoolean("Zero Arm", false);
    // add actual limits
    private final SlewRateLimiter xLimiterL4, yLimiterL4, thetaLimiterL4, xLimiter, yLimiter, thetaLimiter;
    private final CommandXboxController driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(
            OperatorConstants.kOperatorControllerPort);
    private final CommandXboxController emergencyController = new CommandXboxController(
            OperatorConstants.kEmergencyControllerPort);
    private final LoggedTunableNumber ipVbusChar = new LoggedTunableNumber("Infeed Pivot Char Vbus", 0);
    private double currSpeed = DEFAULT_BASE_SPEED;
    @AutoLogOutput
    private boolean climbDeadmanUnsafe = false;
    @AutoLogOutput
    private boolean isSuperCycle = false;
    @AutoLogOutput
    private boolean isAltBarge = false;
    @AutoLogOutput
    private boolean unjamOn = true;

    public RobotContainer() {
        zeroArm.hasChanged(hashCode());
        drive.setPose(new Pose2d(drive.getPose().getTranslation(),
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? Rotation2d.kZero
                        : Rotation2d.kPi));
        xLimiterL4 = new SlewRateLimiter(1.8);
        yLimiterL4 = new SlewRateLimiter(1.8);
        thetaLimiterL4 = new SlewRateLimiter(1.8);
        abortGrondLastScore.stop();
        abortGrondLastScore.reset();

        xLimiter = new SlewRateLimiter(4.0);
        yLimiter = new SlewRateLimiter(4.0);
        thetaLimiter = new SlewRateLimiter(4.0);
        NamedCommands.registerCommand("Lolipop Algae",
                Commands.runOnce(() -> armistice.setSafety(false))
                        .andThen(armistice.runToPositionNoWait(ArmisticePositions.LOLI_ACQUIRE)
                                .alongWith(coral.runMotorCommandAlgae(0.95))
                                .alongWith(Commands.waitUntil(coral.hasAlgae()))
                                .alongWith(infeed.runMotorCommand(.95)
                                        .alongWith(Commands.runOnce(() -> coral.setHasGamepiece(false)))
                                        .alongWith(pivot.runDown().asProxy())))
                        .finallyDo(() -> armistice.setSafety(true)));
        NamedCommands.registerCommand("Loli Acquire Wait", Commands.waitUntil(coral.hasAlgae()));
        NamedCommands.registerCommand("Loli Stage", armistice.runToPositionNoWait(ArmisticePositions.LOLI_ACQUIRE));
        NamedCommands.registerCommand("Set Coral Lock ON",
                Commands.runOnce(() -> drive.setGPVisionCorrection(ll3Coral)));
        NamedCommands.registerCommand("Set Coral Lock OFF", Commands.runOnce(() -> {
            drive.setGPVisionCorrection(null);
            drive.resetGPTY();
        }));
        NamedCommands.registerCommand("Guarentee Stop", realDrivetrainStop());
        NamedCommands.registerCommand("Algae Wait", Commands.waitUntil(coral.hasAlgae()));
        NamedCommands.registerCommand("Acquire Wait", Commands.waitUntil(infeed.hasGamepieceSupplier()));
        NamedCommands.registerCommand("Acquire", ///
                infeed.runMotorCommand(.95).alongWith(Commands.runOnce(() -> coral.setHasGamepiece(false)))
                        .alongWith(pivot.runDown().asProxy()).andThen(
                                Commands.waitUntil(() -> armistice.getTargetPosition() == ArmisticePositions.CLEAN))
                        .andThen(armistice.waitUntilThingsInTolerance(1, Units.degreesToRadians(2))
                                .alongWith(Commands.waitUntil(infeed.hasGamepieceSupplier())))
                        .andThen(coral.runMotorCommand(0.8))
                        .andThen(Commands.runOnce(() -> armistice.setSafety(false)))
                        .andThen(pivot.runUp().asProxy().onlyIf(pivot.isUp().bsnot()))
                        .andThen(Commands.waitUntil(coral.hasGamePieceSupplier()))
                        .andThen(armistice.runToPositionNoWait(ArmisticePositions.STOW).alongWith(
                                Commands.runOnce(() -> infeed.setHasCoral(false))
                                        .alongWith(infeed.directRunMotorCommand(-0.2))))
                        .finallyDo(() -> armistice.waitUntilThingsInTolerance(1, Units.degreesToRadians(5))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(true))
                                        .onlyIf(() -> !MagicSequencing.isMagicScoreRunning))
                                .onlyIf(() -> !MagicSequencing.isMagicScoreRunning).schedule()));
        NamedCommands.registerCommand("Acquire PreL2", ///
                infeed.runMotorCommand(.95).alongWith(Commands.runOnce(() -> coral.setHasGamepiece(false)))
                        .alongWith(pivot.runDown().asProxy()).andThen(
                                Commands.waitUntil(() -> armistice.getTargetPosition() == ArmisticePositions.CLEAN))
                        .andThen(armistice.waitUntilThingsInTolerance(1, Units.degreesToRadians(2))
                                .alongWith(Commands.waitUntil(infeed.hasGamepieceSupplier())))
                        .andThen(coral.runMotorCommand(0.8))
                        .andThen(Commands.runOnce(() -> armistice.setSafety(false)))
                        .andThen(pivot.runUp().asProxy().onlyIf(pivot.isUp().bsnot()))
                        .andThen(Commands.waitUntil(coral.hasGamePieceSupplier()))
                        .andThen(armistice.runToPositionNoWait(ArmisticePositions.Cora_L2).alongWith(
                                Commands.runOnce(() -> infeed.setHasCoral(false))
                                        .alongWith(infeed.directRunMotorCommand(-0.2))))
                        .finallyDo(() -> armistice.waitUntilThingsInTolerance(1, Units.degreesToRadians(5))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(true))
                                        .onlyIf(() -> !MagicSequencing.isMagicScoreRunning))
                                .onlyIf(() -> !MagicSequencing.isMagicScoreRunning).schedule()));
        NamedCommands.registerCommand("Score Outfeed",
                Commands.waitUntil(armistice.armAndElevatorAtTarget())
                        .andThen(Commands.defer(() -> coral.runMotorCommand(getOutfeedVBus()), Set.of(coral))
                                .alongWith(Commands.waitSeconds(0.15))
                                .andThen(coral.runMotorCommand(0))));
        NamedCommands.registerCommand("WaitUntilClose", Commands.waitUntil(drive.driveCloseEnoughReefAuton()));
        NamedCommands.registerCommand("WaitUntilCloseAcqLoliLeft",
                Commands.waitUntil(drive.driveCloseEnoughAcquireAutonLeftLoli()));
        NamedCommands.registerCommand("WaitUntilCloseAcqLoliMid",
                Commands.waitUntil(drive.driveCloseEnoughAcquireAutonMidLoli()));
        NamedCommands.registerCommand("WaitUntilCloseAcqLoliRight",
                Commands.waitUntil(drive.driveCloseEnoughAcquireAutonRightLoli()));
        NamedCommands.registerCommand("WaitUntilCloseAcq", Commands.waitUntil(drive.driveCloseEnoughAcquireAuton()));
        NamedCommands.registerCommand("Run To Closest Right Reef",
                Commands.runOnce(() -> {
                            drive.setReefTargetIsRight(true);
                            infeed.setHasCoral(false);
                        })
                        .andThen(Commands.defer(this::runToClosestReefAuto,
                                Set.of(drive, armistice.getArm(), armistice.getElevator(), coral))));

        NamedCommands.registerCommand("Run To Closest Right Reef Good", Commands.runOnce(() -> {
            drive.setReefTargetIsRight(true);
            infeed.setHasCoral(false);
        }).andThen(Commands.defer(this::runToClosestReefAutoSafe,
                Set.of(drive, armistice.getArm(), armistice.getElevator(), coral))));

        NamedCommands.registerCommand("Run To Closest Right Reef DB",
                Commands.runOnce(() -> {
                            drive.setReefTargetIsRight(true);
                            infeed.setHasCoral(false);
                        })
                        .andThen(Commands.defer(this::runToClosestReefAuto,
                                Set.of(drive, armistice.getArm(), armistice.getElevator(), coral))));
        NamedCommands.registerCommand("Run To Closest Right Reef L2",
                Commands.runOnce(() -> {
                            drive.setReefTargetIsRight(true);
                            infeed.setHasCoral(false);
                        })
                        .andThen(Commands.defer(this::runToClosestReefAutoL2,
                                Set.of(drive, armistice.getArm(), armistice.getElevator(), coral))));
        NamedCommands.registerCommand("Run To Closest Left Reef",
                Commands.runOnce(() -> {
                            drive.setReefTargetIsRight(false);
                            infeed.setHasCoral(false);
                        })
                        .andThen(Commands.defer(this::runToClosestReefAuto,
                                Set.of(drive, armistice.getArm(), armistice.getElevator(), coral))));

        NamedCommands.registerCommand("Run To Closest Left Reef Good", Commands.runOnce(() -> {
            drive.setReefTargetIsRight(false);
            infeed.setHasCoral(false);
        }).andThen(Commands.defer(this::runToClosestReefAutoSafe,
                Set.of(drive, armistice.getArm(), armistice.getElevator(), coral))));

        NamedCommands.registerCommand("Run To Closest Left Reef L2", Commands.runOnce(() -> {
            drive.setReefTargetIsRight(false);
            infeed.setHasCoral(false);
        }).andThen(Commands.defer(this::runToClosestReefAutoL2,
                Set.of(drive, armistice.getArm(), armistice.getElevator(), coral))));

        NamedCommands.registerCommand("SuperCycle Right", Commands.runOnce(() -> {
            drive.setReefTargetIsRight(true);
            infeed.setHasCoral(false);
        }).andThen(Commands.defer(this::runToSuperCycle,
                Set.of(drive, armistice.getArm(), armistice.getElevator(), coral))));

        NamedCommands.registerCommand("SuperCycle Left", Commands.runOnce(() -> {
            drive.setReefTargetIsRight(false);
            infeed.setHasCoral(false);
        }).andThen(Commands.defer(this::runToSuperCycle,
                Set.of(drive, armistice.getArm(), armistice.getElevator(), coral))));

        NamedCommands.registerCommand("Run To Closest Left Reef DB",
                Commands.runOnce(() -> {
                            drive.setReefTargetIsRight(false);
                            infeed.setHasCoral(false);
                        })
                        .andThen(Commands.defer(this::runToClosestReefAuto,
                                Set.of(drive, armistice.getArm(), armistice.getElevator(), coral))));
        NamedCommands.registerCommand("Algae Pickup", Commands.defer(this::runToClosestAlgae,
                Set.of(drive, armistice.getArm(), armistice.getElevator(), coral)));
        NamedCommands.registerCommand("L4 Score",
                runToPositionDeferredClosestReefJSONOffset(() -> ArmisticePositions.Cora_L4_PIPE));
        NamedCommands.registerCommand("Wait TOF", Commands.waitUntil(infeed.hasGamepieceSupplierRawTOF()));
        NamedCommands.registerCommand("Stow",
                armistice.runToPositionCommand(ArmisticePositions.STOW).andThen(coral.runMotorCommand(0)));
        NamedCommands.registerCommand("Stow No Wait", armistice.runToPositionNoWait(ArmisticePositions.STOW));
        NamedCommands.registerCommand("Acquire Pos",
                runToPositionDeferredClosestReefJSONOffset(() -> ArmisticePositions.CLEAN)
                        .alongWith(pivot.runDown()).alongWith(infeed.runMotorCommand(.8)));
        NamedCommands.registerCommand("Infeed Up", pivot.runUp());
        NamedCommands.registerCommand("L3 Score",
                runToPositionDeferredClosestReefJSONOffset(() -> ArmisticePositions.Cora_L3));
        NamedCommands.registerCommand("L2 Score",
                runToPositionDeferredClosestReefJSONOffset(() -> ArmisticePositions.Cora_L2));
        NamedCommands.registerCommand("Blip", Commands.none());
        NamedCommands.registerCommand("Forever", Commands.run(() -> {
        }));
        NamedCommands.registerCommand("Abort Grond", Commands.waitUntil(() -> abortGrondLastScore.get() <= ABORT_THRESHOLD_SEC));
        NamedCommands.registerCommand("Stop Coral", coral.stopMotorCommand());
        NamedCommands.registerCommand("Algae Outfeed", coral.runMotorCommand(-.6).withTimeout(.5));
        NamedCommands.registerCommand("Barge Score", armistice.runToPositionCommand(ArmisticePositions.BARGE));
        NamedCommands.registerCommand("Barge Score No Wait", armistice.runToPositionNoWait(ArmisticePositions.BARGE));
        NamedCommands.registerCommand("Barge Armistice Tolerance",
                armistice.waitUntilThingsInTolerance(1, Units.degreesToRadians(5),
                        () -> armistice.getTargetPosition() == ArmisticePositions.BARGE
                                || armistice.getTargetPosition() == ArmisticePositions.BARGE_CLOSE));
        NamedCommands.registerCommand("Other Armistice Tolerance",
                armistice.waitUntilThingsInTolerance(1, Units.degreesToRadians(5)));
        NamedCommands.registerCommand("Arm Yeat Tolerance",
                armistice.waitUntilThingsInTolerance(18, Units.degreesToRadians(135)));
        NamedCommands.registerCommand("Algae Grond Pos", armistice.runToPositionCommand(ArmisticePositions.GROND));
        NamedCommands.registerCommand("Grond Acquire", coral.runMotorCommandAlgae(.95));
        NamedCommands.registerCommand("Close Barge", armistice.runToPositionNoWait(ArmisticePositions.BARGE_CLOSE));
        autonChooser = new LoggedDashboardChooser<>("Auton Chooser", AutoBuilder.buildAutoChooser());
        autonChooser.addOption("Char drivetrain", drive.feedforwardCharacterization());
        autonChooser.addOption("Char Wheel Radius", drive.wheelRadiusCharacterization());

        // Set up SysId routines
        VisionUtil.bindSimCameras(new Transform3d[]{new Transform3d()});
        configureBindings();
    }

    public void updateArmisticeAutoAlgae() {
        armistice.updateAutoAlgaePos(drive.closestReefTag());
        if (zeroArm.hasChanged(hashCode())) {
            armistice.getArm().setPosition(-0.87);
        }
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
                        .minus(Rotation2d.fromDegrees(ll.getLimelightRobotYaw()))
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
        Logger.recordOutput("Barge Mode", !isAltBarge ? "Std/Inf" : "Opp/Clmb");
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

    public void disableArmisticeArm() {
        armistice.disableArm();
    }

    public void enableArmisticeArm() {
        armistice.enableArm();
    }

    private void configureBindings() {

        new Trigger(DriverStation::isTeleopEnabled).onTrue(infeed.runMotorCommand(0));
        needsUnjam.debounce(0.2).onTrue(setRumble(driverController, 1, RumbleType.kBothRumble, 0.2)
                .finallyDo(() -> setRumble(driverController, 0, RumbleType.kBothRumble, 0).schedule()));

        // ================= //
        /* DRIVER CONTROLLER */
        // ================= //

        // ==============================================
        // DC -- LY/LX/RX: Drive
        // ==============================================
        drive.setDefaultCommand(drive.joystickDrive(
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
        driverController.leftTrigger().and(() -> !climbDeadmanUnsafe)
                .onTrue(infeed.runMotorCommand(.95).onlyIfNoReqs(infeed.hasGamepieceSupplier().bsnot()))
                .onFalse(infeed.runMotorCommand(0).onlyIfNoReqs(infeed.hasGamepieceSupplier().bsnot()));
        driverController.leftTrigger().and(() -> climbDeadmanUnsafe)
                .onTrue(cage.runVbusCommand(-0.7)).onFalse(cage.runVbusCommand(0));
        driverController.b().onTrue(infeed.runMotorCommand(-0.5)).onFalse(infeed.runMotorCommand(0));

        driverController.leftBumper().onTrue(Commands.either(pivot.runDown(), pivot.runUp(), pivot.isUp()));

        // ==============================================
        // Infeed Code
        // ==============================================

        hasGamePiece.onTrue(Commands.defer(() -> Commands.runOnce(() -> coral.setHasGamepiece(false))
                                .andThen(armistice.waitUntilThingsInTolerance(1, Units.degreesToRadians(2))
                                        .alongWith(armistice.waitForArmSlow())
                                        .alongWith(coral.runMotorCommand(0.8)))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(false)))
                                .andThen(pivot.runUp().onlyIf(pivot.isUp().bsnot()))
                                .andThen(Commands.waitUntil(coral.hasGamePieceSupplier()).alongWith(Commands.waitUntil(
                                                () -> InfeedPivotPositions.HANDOFF.posRad - pivot.getPosition() < Units.degreesToRadians(-2.))
                                        .andThen(infeed.directRunMotorCommand(-0.4))))
                                .andThen(Commands.runOnce(() -> infeed.setHasCoral(false)))
                                .andThen(armistice.runToPositionNoWait(ArmisticePositions.STOW)),
                        Set.of(armistice.getArm(), armistice.getElevator(), coral, pivot, infeed)).finallyDo(() -> {
                    armistice.runToPositionNoWait(ArmisticePositions.STOW)
                            .onlyIf(() -> armistice.getTargetPosition() == ArmisticePositions.CLEAN).andThen(
                                    armistice.waitUntilThingsInTolerance(1, Units.degreesToRadians(5)),
                                    Commands.runOnce(() -> armistice.setSafety(true))
                                            .onlyIf(() -> !MagicSequencing.isMagicScoreRunning))
                            .onlyIf(() -> !MagicSequencing.isMagicScoreRunning).andThen(infeed.runMotorCommand(0))
                            .schedule();
                    infeed.setHasCoral(false);
                })
                .onlyIfNoReqs(DriverStation::isTeleop));

        // ==============================================
        // DC -- LB: Outfeed Coral
        // ==============================================

        driverController.rightBumper().onTrue(Commands.runOnce(() -> currSpeed = SLOW_SPEED))
                .onFalse(Commands.runOnce(() -> currSpeed = DEFAULT_BASE_SPEED));

        driverController.a().onTrue(cage.runVbusCommand(-0.7)).onFalse(cage.runVbusCommand(0));

        // =================== //
        /* OPERATOR CONTROLLER */
        // =================== //

        // ==============================================
        // OC -- START: Toggle Barge Mode
        // ==============================================
        operatorController.start().onTrue(Commands.runOnce(() -> isAltBarge = !isAltBarge).ignoringDisable(true));

        // ==============================================
        // OC -- RB: Magic Score Right Branch
        // ==============================================
        operatorController.rightBumper().onTrue(
                Commands.runOnce(() -> drive.setReefTargetIsRight(true)).andThen(Commands.defer(this::runToClosestReef,
                                Set.of(drive, armistice.getArm(), armistice.getElevator(), coral)))
                        .onlyIf(() -> !climbDeadmanUnsafe));

        // ==============================================
        // OC -- LB: Magic Score Left Branch
        // ==============================================
        operatorController.leftBumper().onTrue(
                Commands.runOnce(() -> drive.setReefTargetIsRight(false)).andThen(Commands.defer(this::runToClosestReef,
                                Set.of(drive, armistice.getArm(), armistice.getElevator(), coral)))
                        .onlyIf(() -> !climbDeadmanUnsafe));

        // ==============================================
        // OC -- Back: Toggle climb deadman
        // ==============================================
        // operatorController.back()
        // .onTrue(Commands.runOnce(() -> isSuperCycle =
        // !isSuperCycle).ignoringDisable(true));

        // ==============================================
        // OC -- LT: Infeed Algae
        // ==============================================
        operatorController.leftTrigger().onTrue(coral.runMotorCommandAlgae(0.95))
                .onFalse(coral.runMotorCommand(0));

        // ==============================================
        // OC -- RT: Outfeed Algae
        // ==============================================
        operatorController.rightTrigger()
                .onTrue(Commands.defer(
                        () -> coral.runMotorCommand(coral.hasAlgae().getAsBoolean() ? -0.6 : /* L1 */ -0.1),
                        Set.of(coral)))
                .onFalse(coral.runMotorCommand(0));

        // ==============================================
        // OC -- LY: Climber (up = climb, down = slow)
        // ==============================================
        operatorController.axisLessThan(XboxController.Axis.kLeftY.value, 0.5).onTrue(climber.runVbusCommand(0));
        operatorController.axisLessThan(XboxController.Axis.kLeftY.value, -0.8)
                .onTrue(climber.climbFastCommand().onlyIf(() -> climbDeadmanUnsafe));
        operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.5)
                .onTrue(armistice.runToPositionCommand(ArmisticePositions.GROND).onlyIf(() -> !climbDeadmanUnsafe));
        operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.5)
                .onTrue(climber.runVbusCommand(0.2).onlyIf(() -> climbDeadmanUnsafe));

        // ==============================================
        // OC -- DPAD UP: Increment Armistice Manual Index
        // ==============================================
        operatorController.povUp().onTrue(armistice.incFutureArmisticePosition());

        // ==============================================
        // OC -- DPAD DOWN: Decrement Armistice Manual Index
        // ==============================================
        operatorController.povDown().onTrue(armistice.decFutureArmisticePosition());

        // ==============================================
        // OC -- DPAD RIGHT: Run to Climb
        // ==============================================
        operatorController.povRight()
                .onTrue(armistice.runToPositionNoWait(ArmisticePositions.CLIMB).alongWith(pivot.runUpClimb())
                        .alongWith(climber.deployCommand().onlyIf(() -> !hasDeployed), Commands.runOnce(() -> hasDeployed = true))
                        .onlyIfNoReqs(() -> climbDeadmanUnsafe));

        // ==============================================
        // OC -- DPAD LEFT: Climber
        // ==============================================
        operatorController.povLeft().onTrue(climber.climbSlowCommand().onlyIf(() -> climbDeadmanUnsafe));

        operatorController.back()
                .onTrue(Commands.runOnce(() -> climbDeadmanUnsafe = !climbDeadmanUnsafe).ignoringDisable(true));

        // ==============================================
        // OC -- Y: Run To Aquire
        // ==============================================
        operatorController.y()
                .onTrue(Commands
                        .defer(() -> Commands.runOnce(() -> {
                                            armistice.setSafety(false);
                                            coral.setHasGamepiece(false);
                                        }).andThen(armistice.runToPositionCommand(ArmisticePositions.CLEAN,
                                                drive.closestReefName(), drive.getReefTargetIsRight()))
                                        .finallyDo(() -> armistice.waitUntilThingsInTolerance(1, Units.degreesToRadians(5))
                                                .andThen(Commands.runOnce(() -> armistice.setSafety(true)))),
                                Set.of(armistice.getArm(), armistice.getElevator()))
                        .alongWith(pivot.runDown())
                        .onlyIf(() -> !climbDeadmanUnsafe));

        // ==============================================
        // OC -- A: Run to ground algae
        // ==============================================
        operatorController.a()
                .onTrue(Commands.runOnce(() -> armistice.setSafety(false))
                        .andThen(armistice.runToPositionCommand(ArmisticePositions.GROND, drive.closestReefName(),
                                drive.getReefTargetIsRight())));
        new Trigger(() -> armistice.getTargetPosition() == ArmisticePositions.GROND)
                .onFalse(Commands.runOnce(() -> armistice.setSafety(true)));

        // ==============================================
        // OC -- X: Run To Stow
        // ==============================================
        operatorController.x()
                .onTrue(armistice.runToPositionCommand(ArmisticePositions.STOW)
                        .onlyIf(() -> !climbDeadmanUnsafe));

        // ==============================================
        // OC -- B: Magic Score Algae
        // ==============================================
        operatorController.b().onTrue(Commands
                .defer(this::runToClosestAlgae, Set.of(drive, armistice.getArm(), armistice.getElevator(), coral))
                .onlyIf(() -> !climbDeadmanUnsafe));

        // ==============================================
        // OC -- RS UP: Run to Barge
        // ==============================================
        operatorController.axisLessThan(XboxController.Axis.kRightY.value, -0.5).onTrue(Commands.defer(() -> armistice
                        .runToPositionCommand(isAltBarge ? ArmisticePositions.BARGE_OPPOSITE : ArmisticePositions.BARGE),
                Set.of(armistice.getArm(), armistice.getElevator())));

        // ==============================================
        // OC -- RS DOWN: Run to Proc
        // ==============================================
        operatorController.axisGreaterThan(XboxController.Axis.kRightY.value, 0.5)
                .onTrue(Commands.runOnce(() -> armistice.setSafety(false))
                        .andThen(armistice.runToPositionCommand(ArmisticePositions.PROC))
                        .finallyDo(() -> armistice.setSafety(true)));

        // ==================== //
        /* EMERGENCY CONTROLLER */
        // ==================== //
        if (Constants.CHAR_MODE) {
            emergencyController.y().whileTrue(armistice.sysIDCommandElevator(() -> false, () -> Direction.kForward));
            emergencyController.a().whileTrue(armistice.sysIDCommandElevator(() -> false, () -> Direction.kReverse));
            emergencyController.povUp().whileTrue(armistice.sysIDCommandElevator(() -> true, () -> Direction.kForward));
            emergencyController.povDown()
                    .whileTrue(armistice.sysIDCommandElevator(() -> true, () -> Direction.kReverse));
            emergencyController.start().onTrue(armistice.runArmVoltageForChar());
            emergencyController.back().onTrue(armistice.stopArm());
            emergencyController.rightBumper()
                    .onTrue(Commands.defer(() -> pivot.runMotorCommand(ipVbusChar.get()), Set.of(pivot)))
                    .onFalse(pivot.runMotorCommand(0));
            emergencyController.b().onTrue(pivot.runToPositionCommand(InfeedPivotPositions.UP.posRad));
            emergencyController.x().onTrue(pivot.runMotorCommand(-0.2));
            emergencyController.leftTrigger().onTrue(pivot.runToPositionCommand(InfeedPivotPositions.HANDOFF.posRad));
            emergencyController.axisMagnitudeGreaterThan(XboxController.Axis.kLeftX.value,
                            0.5)
                    .onTrue(Commands.runOnce(() -> drive.setReefTargetIsRight(
                                    Math.signum(emergencyController.getRawAxis(XboxController.Axis.kLeftX.value)) > 0))
                            .ignoringDisable(true));
        } else {
            // ==============================================
            // EC -- DPAD: Global Nudges
            // ==============================================
            emergencyController.povUp().onTrue(armistice.nudgeCommandGlobalPermanent(1,
                    0));
            emergencyController.povDown().onTrue(armistice.nudgeCommandGlobalPermanent(-1,
                    0));
            emergencyController.povRight().onTrue(armistice.nudgeCommandGlobalPermanent(0,
                    Units.degreesToRadians(1)));
            emergencyController.povLeft().onTrue(armistice.nudgeCommandGlobalPermanent(0,
                    Units.degreesToRadians(-1)));

            // ==============================================
            // EC -- DPAD: Positional Nudges
            // ==============================================
            emergencyController.y().onTrue(armistice.nudgeCommandPermanent(1, 0));
            emergencyController.a().onTrue(armistice.nudgeCommandPermanent(-1, 0));
            emergencyController.b().onTrue(armistice.nudgeCommandPermanent(0,
                    Units.degreesToRadians(1)));
            emergencyController.x().onTrue(armistice.nudgeCommandPermanent(0,
                    Units.degreesToRadians(-1)));

            emergencyController.rightStick().onTrue(armistice.resetNudges().ignoringDisable(true));

            emergencyController.axisMagnitudeGreaterThan(XboxController.Axis.kRightX.value,
                            0.5)
                    .onTrue(armistice.toggleCoralReefOffset()
                            .alongWith(setRumble(operatorController, 1, RumbleType.kBothRumble, 0.2))
                            .ignoringDisable(true));

            emergencyController.axisMagnitudeGreaterThan(XboxController.Axis.kLeftX.value,
                            0.5)
                    .onTrue(Commands.runOnce(() -> drive.setReefTargetIsRight(
                                    Math.signum(emergencyController.getRawAxis(XboxController.Axis.kLeftX.value)) > 0))
                            .ignoringDisable(true));

            emergencyController.start().onTrue(
                    armistice.runToFutureArmisticePositionCommand(drive::closestReefName,
                                    drive::getReefTargetIsRight)
                            .onlyIf(() -> !climbDeadmanUnsafe));

            emergencyController.back()
                    .onTrue(Commands.runOnce(() -> humanCam.setCamera(climbDeadmanUnsafe = !climbDeadmanUnsafe))
                            .ignoringDisable(true));
            emergencyController.rightTrigger().onTrue(climber.deployCommand());
            emergencyController.leftTrigger()
                    .onTrue(climber.runPositionCommand(ClimberConstants.ClimberPositions.CLIMB));
            emergencyController.leftBumper().onTrue(Commands.runOnce(() -> drive.setReefTargetIsRight(false))
                    .andThen(Commands.defer(this::runToClosestReefEmergency,
                            Set.of(drive, armistice.getArm(), armistice.getElevator(), coral)))
                    .onlyIf(() -> !climbDeadmanUnsafe));
            emergencyController.rightBumper().onTrue(Commands.runOnce(() -> drive.setReefTargetIsRight(true))
                    .andThen(Commands.defer(this::runToClosestReefEmergency,
                            Set.of(drive, armistice.getArm(), armistice.getElevator(), coral)))
                    .onlyIf(() -> !climbDeadmanUnsafe));
        }
    }

    public void startAutonTimer() {
        abortGrondLastScore.restart();
    }

    public Command getAutonomousCommand() {
        return autonChooser.get().finallyDo(() -> drive.setGPVisionCorrection(null));
    }

    public double chooseXLimiter(double input) {
        if (armistice.getElevatorPosition() > 40) {
            return xLimiterL4.calculate(input);
        } else {
            return xLimiter.calculate(input);
        }

    }

    public double chooseYLimiter(double input) {
        if (armistice.getElevatorPosition() > 40) {
            return yLimiterL4.calculate(input);
        } else {
            return yLimiter.calculate(input);
        }
    }

    public double chooseThetaLimiter(double input) {
        if (armistice.getElevatorPosition() > 40) {
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
                : MagicSequencing.magicScore(drive, armistice, coral, drive::pipe1ClosestReefPose,
                        () -> DriverStation.isAutonomous() ? ArmisticePositions.Cora_L4
                                : armistice.getFutureArmisticePositions(),
                        () -> isSuperCycle).andThen(pivot.runDown().asProxy())
                .andThen(MagicSequencing
                        .magicAlgae(drive, armistice, coral, drive::pipe1AlgaeClosestReefPose,
                                armistice::getAutoAlgaePosition, () -> isSuperCycle)
                        .finallyDo(() -> isSuperCycle = false)
                        .asProxy().onlyIf(() -> isSuperCycle));
    }

    private Command runToSuperCycle() {
        return MagicSequencing.magicScoreNoStow(drive, armistice, coral, drive::pipe1ClosestReefPose,
                        () -> DriverStation.isAutonomous() ? ArmisticePositions.Cora_L4_PIPE
                                : armistice.getFutureArmisticePositions(),
                        () -> true).andThen(pivot.runDown().asProxy())
                .andThen(MagicSequencing
                        .magicAlgae(drive, armistice, coral, drive::pipe1AlgaeClosestReefPose,
                                armistice::getAutoAlgaePosition, () -> true));

    }

    private Command runToClosestReefEmergency() {
        return armistice.magicIsSnap() ? magicSnapL1()
                : MagicSequencing.magicScore(drive, armistice, coral, drive::closestReefPose,
                        () -> DriverStation.isAutonomous() ? ArmisticePositions.Cora_L4
                                : armistice.getFutureArmisticePositions().getEmergencyPosition(),
                        () -> isSuperCycle).andThen(pivot.runDown().asProxy())
                .andThen(MagicSequencing
                        .magicAlgae(drive, armistice, coral, drive::closestReefPoseAlgae,
                                () -> armistice.getAutoAlgaePosition().getEmergencyPosition(),
                                () -> isSuperCycle)
                        .finallyDo(() -> isSuperCycle = false)
                        .asProxy().onlyIf(() -> isSuperCycle));
    }

    private Command runToClosestReefAuto() {
        return MagicSequencing.magicScoreNoBackup(drive, armistice, coral, drive::pipe1ClosestReefPose,
                () -> ArmisticePositions.Cora_L4);
    }

    private Command runToClosestReefAutoSafe() {
        return MagicSequencing.magicScoreSafeNoBackup(drive, armistice, coral, drive::pipe1ClosestReefPose,
                () -> ArmisticePositions.Cora_L4);
    }

    private Command runToClosestReefAutoL2() {
        return MagicSequencing.magicScoreL2NoBackup(drive, armistice, coral, drive::pipe1ClosestReefPose,
                () -> ArmisticePositions.Cora_L2);
    }

    private Command magicSnapL1() {
        return armistice.runToPositionNoWait(ArmisticePositions.Cora_L1);
    }

    private Command runToClosestAlgae() {
        return Commands.defer(
                () -> MagicSequencing.magicAlgae(drive, armistice, coral, drive::pipe1AlgaeClosestReefPose,
                        armistice::getAutoAlgaePosition, () -> isSuperCycle),
                Set.of(drive, armistice.getArm(), armistice.getElevator(), coral));
    }

    private Command runToClosestAlgaeEmergency() {
        return Commands.defer(
                () -> MagicSequencing.magicAlgae(drive, armistice, coral, drive::closestReefPoseAlgae,
                        () -> armistice.getAutoAlgaePosition().getUnEmergencyPosition(), () -> isSuperCycle),
                Set.of(drive, armistice.getElevator(), armistice.getArm(), coral));
    }

    private double scaleDriverController(DoubleSupplier controllerInput, LimiterState type) {
        double input = controllerInput.getAsDouble() * ((currSpeed)
                + (currSpeed == SLOW_SPEED ? 0 : driverController.getRightTriggerAxis() * (1 - currSpeed)));
        return switch (type) {
            case X -> chooseXLimiter(input);
            case Y -> chooseYLimiter(input);
            case THETA -> chooseThetaLimiter(input);
        };
    }

    private Command setRumble(CommandXboxController controller, double amnt, RumbleType type, double time) {
        return Commands.runOnce(() -> controller.setRumble(type, amnt)).andThen(Commands.waitSeconds(time))
                .andThen(Commands.runOnce(() -> controller.setRumble(type, 0)));
    }

    private double getOutfeedVBus() {
        return armistice.getElevatorPosition() > 45 ? -.85
                : armistice.getTargetPosition() == ArmisticePositions.Cora_L1 ? -.35
                : -.4;
    }

    public Command realDrivetrainStop() {
        return drive.runOnce(drive::stop);
    }

    public enum LimiterState {
        X,
        Y,
        THETA
    }

}
