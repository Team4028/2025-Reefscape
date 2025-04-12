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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Armistice.ArmisticePositions;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MagicSequencing;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
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
import frc.robot.subsystems.infeedpivot.InfeedPivotMotorIOTalonFX;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.limelight.LimelightConstants;
import frc.robot.subsystems.limelight.LimelightIO;
import frc.robot.subsystems.limelight.LimelightIO.LoggablePoseEstimate;
import frc.robot.subsystems.stick.WhipStick;
import frc.robot.subsystems.stick.WhipStickIOTalonFX;
import frc.robot.util.LoggedTunables.LoggedTunableNumber;
import frc.robot.util.MiscUtils;
import frc.robot.util.RobotSim;
import frc.robot.util.VisionUtil;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({ frc.robot.util.RobotSim.class, frc.robot.commands.DriveCommands.class, MiscUtils.class })
public class RobotContainer {
    public enum LimiterState {
        X,
        Y,
        THETA
    }

    private final WhipStick coral = new WhipStickIOTalonFX().simSwitch();
    private final Armistice armistice = new Armistice(coral.hasAlgae());
    private final Climber climber = new ClimberIOTalonFX().simSwitch();
    private final Grond infeed = new Grond(new GrondIOTalonFX(), new GrondTOFIOPWF());
    private final InfeedPivot pivot = new InfeedPivot(new InfeedPivotMotorIOTalonFX(),
            new InfeedPivotEncoderIOCancoder());

    private final Limelight ll4iii = new Limelight(new LimelightIO("limelight-fouriii", true, Optional.empty()));
    private final Limelight ll4ii = new Limelight(new LimelightIO("limelight-fourii", true, Optional.empty()));

    private static final double SLOW_SPEED = 0.2;
    private static final double DEFAULT_BASE_SPEED = 0.3;
    private double currSpeed = DEFAULT_BASE_SPEED;

    @AutoLogOutput
    private boolean isRRelative = false;

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

    @AutoLogOutput
    private boolean isSuperCycle = false;

    private final Trigger hasGamePiece = new Trigger(
            infeed.hasGamepieceSupplier().and(() -> armistice.getTargetPosition() == ArmisticePositions.CLEAN)
                    .and(pivot.isUp().not()));

    @AutoLogOutput
    private final Trigger supercycleIsL4 = new Trigger(
            () -> armistice.getFutureArmisticePositions() == ArmisticePositions.Cora_L4);

    @AutoLogOutput
    private final Trigger scIsGood = new Trigger(() -> (armistice.getFutureArmisticePositions().isCoralScore()
            && armistice.getFutureArmisticePositions() != ArmisticePositions.Cora_L1)
            && (armistice.getTargetPosition() != ArmisticePositions.Cora_L4
                    || drive.driveCloseEnoughReefAuton().getAsBoolean()));

    @AutoLogOutput
    private final Trigger sensesPipeMagicScore = new Trigger(drive.hasPipeAtReef(armistice));

    // add actual limits
    private final SlewRateLimiter xLimiterL4, yLimiterL4, thetaLimiterL4, xLimiter, yLimiter, thetaLimiter;

    private final CommandXboxController driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(
            OperatorConstants.kOperatorControllerPort);
    private final CommandXboxController emergencyController = new CommandXboxController(
            OperatorConstants.kEmergencyControllerPort);

    private final LoggedTunableNumber ipVbusChar = new LoggedTunableNumber("Infeed Pivot Char Vbus", 0);

    public RobotContainer() {
        pivot.zero();
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
        NamedCommands.registerCommand("Acquire", infeed.runMotorCommand(.8).alongWith(pivot.runDown()).andThen(
                Commands.waitUntil(() -> armistice.getTargetPosition() == ArmisticePositions.CLEAN))
                .andThen(armistice.waitUntilThingsInTolerance(1, 0.1))
                .alongWith(Commands.waitUntil(infeed.hasGamepieceSupplier()))
                .andThen(coral.runMotorCommand(0.5))
                .andThen(Commands.runOnce(() -> armistice.setSafety(false)))
                .andThen(pivot.runUp().onlyIf(pivot.isUp().not()))
                .andThen(Commands.waitUntil(coral.hasGamePieceSupplier()))
                .andThen(armistice.runToPositionNoWait(ArmisticePositions.STOW).alongWith(
                        Commands.runOnce(() -> infeed.setHasCoral(false))
                                .alongWith(infeed.runMotorCommand(0))))
                .finallyDo(() -> {
                    armistice.waitUntilThingsInTolerance(1, Units.degreesToRadians(5))
                            .andThen(Commands.runOnce(() -> armistice.setSafety(true))).schedule();
                }));
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
                Commands.runOnce(() -> drive.setReefTargetIsRight(true))
                        .andThen(Commands.defer(this::runToClosestReefAuto,
                                Set.<Subsystem>of(drive, armistice.getArm(), armistice.getElevator(), coral))));
        NamedCommands.registerCommand("Run To Closest Right Reef DB",
                Commands.runOnce(() -> drive.setReefTargetIsRight(true))
                        .andThen(Commands.defer(this::runToClosestReefAuto,
                                Set.<Subsystem>of(drive, armistice.getArm(), armistice.getElevator(), coral))));
        NamedCommands.registerCommand("Run To Closest Right Reef L2",
                Commands.runOnce(() -> drive.setReefTargetIsRight(true))
                        .andThen(Commands.defer(this::runToClosestReefAutoL2,
                                Set.<Subsystem>of(drive, armistice.getArm(), armistice.getElevator(), coral))));
        NamedCommands.registerCommand("Run To Closest Left Reef",
                Commands.runOnce(() -> drive.setReefTargetIsRight(false))
                        .andThen(Commands.defer(this::runToClosestReefAuto,
                                Set.<Subsystem>of(drive, armistice.getArm(), armistice.getElevator(), coral))));
        NamedCommands.registerCommand("Run To Closest Left Reef L2",
                Commands.runOnce(() -> drive.setReefTargetIsRight(false))
                        .andThen(Commands.defer(this::runToClosestReefAutoL2,
                                Set.<Subsystem>of(drive, armistice.getArm(), armistice.getElevator(), coral))));
        NamedCommands.registerCommand("Run To Closest Left Reef DB",
                Commands.runOnce(() -> drive.setReefTargetIsRight(false))
                        .andThen(Commands.defer(this::runToClosestReefAuto,
                                Set.<Subsystem>of(drive, armistice.getArm(), armistice.getElevator(), coral))));
        NamedCommands.registerCommand("L4 Score",
                runToPositionDeferredClosestReefJSONOffset(() -> ArmisticePositions.Cora_L4_PIPE));
        NamedCommands.registerCommand("Stow", armistice.runToPositionCommand(ArmisticePositions.STOW));
        NamedCommands.registerCommand("Stow No Wait", armistice.runToPositionNoWait(ArmisticePositions.STOW));
        NamedCommands.registerCommand("Acquire Pos",
                runToPositionDeferredClosestReefJSONOffset(() -> ArmisticePositions.CLEAN)
                        .alongWith(infeed.runMotorCommand(0.55)).alongWith(pivot.runDown()));
        NamedCommands.registerCommand("L3 Score",
                runToPositionDeferredClosestReefJSONOffset(() -> ArmisticePositions.Cora_L3));
        NamedCommands.registerCommand("L2 Score",
                runToPositionDeferredClosestReefJSONOffset(() -> ArmisticePositions.Cora_L2));
        NamedCommands.registerCommand("Blip", Commands.none());
        autonChooser = new LoggedDashboardChooser<>("Auton Chooser", AutoBuilder.buildAutoChooser());
        autonChooser.addOption("Char drivetrain", drive.feedforwardCharacterization());
        autonChooser.addOption("Char Wheel Radius", drive.wheelRadiusCharacterization())

        // Set up SysId routines
        ;
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

    private void configureBindings() {

        // ================= //
        /* DRIVER CONTROLLER */
        // ================= //

        // ==============================================
        // DC -- LY/LX/RX: Drive
        // ==============================================
        drive.setDefaultCommand(
                Commands.either(drive.joystickDrive(
                        () -> scaleDriverController(() -> -driverController.getLeftY(), LimiterState.X),
                        () -> scaleDriverController(() -> -driverController.getLeftX(), LimiterState.Y),
                        () -> scaleDriverController(() -> -driverController.getRightX(),
                                LimiterState.THETA)),
                        drive.joystickDriveRR(
                                () -> scaleDriverController(() -> -driverController.getLeftY(), LimiterState.X),
                                () -> scaleDriverController(() -> -driverController.getLeftX(), LimiterState.Y),
                                () -> scaleDriverController(() -> -driverController.getRightX(), LimiterState.THETA)),
                        () -> !isRRelative));

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

        driverController.y().onTrue(Commands.runOnce(() -> isRRelative = !isRRelative));

        // ==============================================
        // DC -- LT: Infeed Coral
        // ==============================================
        driverController.leftTrigger()
                .onTrue(infeed.runMotorCommand(.8).onlyIfNoReqs(infeed.hasGamepieceSupplier().not()))
                .onFalse(infeed.runMotorCommand(0).onlyIfNoReqs(infeed.hasGamepieceSupplier().not()));
        driverController.b().onTrue(infeed.runMotorCommand(-0.5)).onFalse(infeed.runMotorCommand(0));

        driverController.leftBumper().onTrue(Commands.either(pivot.runDown(), pivot.runUp(), pivot.isUp()));

        hasGamePiece.onTrue(Commands.defer(() -> armistice.waitUntilThingsInTolerance(1, 0.1)
                .alongWith(coral.runMotorCommand(0.5))
                .andThen(Commands.runOnce(() -> armistice.setSafety(false)))
                .andThen(pivot.runUp().onlyIf(pivot.isUp().not()))
                .andThen(Commands.waitUntil(coral.hasGamePieceSupplier()))
                .andThen(armistice.runToPositionNoWait(ArmisticePositions.STOW).alongWith(
                        Commands.runOnce(() -> infeed.setHasCoral(false))
                                .alongWith(infeed.runMotorCommand(0))))
                .finallyDo(() -> {
                    armistice.waitUntilThingsInTolerance(1, Units.degreesToRadians(5))
                            .andThen(Commands.runOnce(() -> armistice.setSafety(true))).schedule();
                }),
                Set.of(armistice.getArm(), armistice.getElevator(), coral, pivot, infeed))
                .asProxy().onlyIfNoReqs(DriverStation::isTeleop));

        // ==============================================
        // DC -- LB: Outfeed Coral
        // ==============================================

        driverController.rightBumper().onTrue(Commands.runOnce(() -> currSpeed = SLOW_SPEED))
                .onFalse(Commands.runOnce(() -> currSpeed = DEFAULT_BASE_SPEED));

        // ==============================================
        // OC -- START: Toggle Game Piece Mode
        // ==============================================
        // operatorController.start()
        // .onTrue(Commands
        // .either(armistice.runToPositionCommand(ArmisticePositions.CLIMB_2),
        // armistice.runToPositionCommand(ArmisticePositions.CLIMB),
        // () -> armistice.getTargetPosition() == ArmisticePositions.CLIMB)
        // .onlyIf(() -> climbDeadmanUnsafe));
        operatorController.start().onTrue(Commands.runOnce(() -> isSuperCycle = !isSuperCycle).ignoringDisable(true));
        operatorController.povRight()
                .onTrue(armistice.runToPositionCommand(ArmisticePositions.CLIMB_2).onlyIf(() -> climbDeadmanUnsafe));
        operatorController.povLeft()
                .onTrue(armistice.runToPositionCommand(ArmisticePositions.CLIMB).onlyIf(() -> climbDeadmanUnsafe));

        // =================== //
        /* OPERATOR CONTROLLER */
        // =================== //

        // ==============================================
        // OC -- RB: Magic Score Right Branch
        // ==============================================
        operatorController.rightBumper().onTrue(
                Commands.runOnce(() -> drive.setReefTargetIsRight(true)).andThen(Commands.defer(this::runToClosestReef,
                        Set.<Subsystem>of(drive, armistice.getArm(), armistice.getElevator(), coral)))
                        .onlyIf(() -> !climbDeadmanUnsafe));

        // ==============================================
        // OC -- LB: Magic Score Left Branch
        // ==============================================
        operatorController.leftBumper().onTrue(
                Commands.runOnce(() -> drive.setReefTargetIsRight(false)).andThen(Commands.defer(this::runToClosestReef,
                        Set.<Subsystem>of(drive, armistice.getArm(), armistice.getElevator(), coral)))
                        .onlyIf(() -> !climbDeadmanUnsafe));

        operatorController.back()
                .onTrue(Commands.runOnce(() -> climbDeadmanUnsafe = !climbDeadmanUnsafe).ignoringDisable(true));

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
        // operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value,
        // 0.5).onTrue(climber.runVbusCommand(-0.4));
        operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value, -0.5)
                .onTrue(climber.runVbusCommand(0));
        operatorController.axisLessThan(XboxController.Axis.kLeftY.value, 0.5).onTrue(climber.runVbusCommand(0));
        operatorController.axisLessThan(XboxController.Axis.kLeftY.value, -0.5)
                .onTrue(climber.runVbusCommand(0.7).onlyIf(() -> climbDeadmanUnsafe));
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
        // OC -- DPAD LEFT/RIGHT: Inc/Dev Magic Score Algae Height
        // ==============================================

        // ==============================================
        // OC -- Y: Run To Aquire/Lollipop
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
        // OC -- A: Run To Manual Index Position
        // ==============================================
        operatorController.a()
                .onTrue(Commands.runOnce(() -> armistice.setSafety(false))
                        .andThen(armistice.runToPositionCommand(ArmisticePositions.GROND, drive.closestReefName(),
                                drive.getReefTargetIsRight()))
                        .finallyDo(() -> armistice.setSafety(true)));

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

        operatorController.axisLessThan(XboxController.Axis.kRightY.value, -0.5).onTrue(armistice
                .runToPositionCommand(ArmisticePositions.BARGE, drive.closestReefName(), drive.getReefTargetIsRight()));

        operatorController.axisGreaterThan(XboxController.Axis.kRightY.value, 0.5)
                .onTrue(armistice.runToPositionCommand(ArmisticePositions.PROC));

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
        } else {
            // emergencyController.leftBumper().onTrue(coral.runMotorCommand(-0.5)).onFalse(coral.runMotorCommand(0));
            // emergencyController.rightBumper().onTrue(coral.runMotorCommand(0.5)).onFalse(coral.runMotorCommand(0));
            // ==============================================
            // EC -- DPAD: Global Nudges
            // ==============================================
            emergencyController.povUp().onTrue(armistice.nudgeCommandGlobalPermanant(1,
                    0));
            emergencyController.povDown().onTrue(armistice.nudgeCommandGlobalPermanant(-1,
                    0));
            emergencyController.povRight().onTrue(armistice.nudgeCommandGlobalPermanant(0,
                    Units.degreesToRadians(1)));
            emergencyController.povLeft().onTrue(armistice.nudgeCommandGlobalPermanant(0,
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
            emergencyController.leftStick().onTrue(
                    armistice.runToFutureArmisticePositionCommand(drive::closestReefName,
                            drive::getReefTargetIsRight)
                            .onlyIf(() -> !climbDeadmanUnsafe));

            emergencyController.leftTrigger().onTrue(armistice.toggleCoralMode());

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

            emergencyController.rightBumper()
                    .onTrue(Commands.runOnce(() -> humanCam.setCamera(climbDeadmanUnsafe = !climbDeadmanUnsafe))
                            .ignoringDisable(true));

            emergencyController.leftBumper().onTrue(NamedCommands.getCommand("Acquire"));
        }
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
                : MagicSequencing.magicScoreScore(drive, armistice, coral, drive::pipe1ClosestReefPose,
                        () -> DriverStation.isAutonomous() ? ArmisticePositions.Cora_L4
                                : armistice.getFutureArmisticePositions(),
                        () -> isSuperCycle)
                        .andThen(MagicSequencing
                                .magicGetAlgaeOnlyPID(drive, armistice, coral, drive::pipe1AlgaeClosestReefPose,
                                        armistice::getAutoAlgaePosition, () -> isSuperCycle)
                                .finallyDo(() -> isSuperCycle = false)
                                .asProxy().onlyIf(() -> isSuperCycle));
    }

    private Command runToClosestReefAuto() {
        return MagicSequencing.magicScoreNoDB(drive, armistice, coral, drive::pipe1ClosestReefPose,
                () -> ArmisticePositions.Cora_L4);
    }

    private Command runToClosestReefAutoL2() {
        return MagicSequencing.magicScoreNoDB(drive, armistice, coral, drive::pipe1ClosestReefPose,
                () -> ArmisticePositions.Cora_L2);
    }

    private Command magicSnapL1() {
        return armistice.runToPositionNoWait(ArmisticePositions.Cora_L1).alongWith(drive.joystickDriveAtAngle(
                () -> scaleDriverController(() -> -driverController.getLeftY(), LimiterState.X),
                () -> scaleDriverController(() -> -driverController.getLeftX(), LimiterState.Y),
                drive::closestReefL1Rotation));
    }

    private Command runToClosestAlgae() {
        return Commands.defer(
                () -> MagicSequencing.magicGetAlgaeOnlyPID(drive, armistice, coral, drive::pipe1AlgaeClosestReefPose,
                        armistice::getAutoAlgaePosition, () -> isSuperCycle),
                Set.of(drive, armistice.getArm(), armistice.getElevator(), coral));
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

    private Command setRumble(CommandXboxController controller, double amnt, RumbleType type, double time) {
        return Commands.runOnce(() -> controller.setRumble(type, amnt)).andThen(Commands.waitSeconds(time))
                .andThen(Commands.runOnce(() -> controller.setRumble(type, 0)));
    }

    private double getOutfeedVBus() {
        return armistice.getElevatorPosition() > 45 ? -.85
                : armistice.getTargetPosition() == ArmisticePositions.Cora_L1 ? -.35
                        : (armistice.getTargetPosition().isPipe() ? -.4 : -.4);
    }

    public Command realDrivetrainStop() {
        return drive.runOnce(drive::stop);
    }

}
