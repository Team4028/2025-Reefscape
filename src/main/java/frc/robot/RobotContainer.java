// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Armistice.ArmisticePositions;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algae.*;
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

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
    public enum LimiterState {
        X,
        Y,
        THETA
    }

    private final CoralManipulator coralManipulator = RobotSim
            .coralManipulatorSimSwitch(new CoralManipulatorIOTalonSRX());

    private final Drive drive = RobotSim.driveSimSwitch(new GyroIOPigeon2(),
            new ModuleIO[] { new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                    new ModuleIOTalonFX(TunerConstants.BackRight) });

    private final AlgaeManipulator algae = RobotSim.algaeSimSwitch(new AlgaeManipulatorIOTalonSRX());
    private final Armistice armistice = new Armistice();
    private final Limelight ll4 = new Limelight(new LimelightIO("limelight-fourii", true, Optional.empty(),
            new Transform3d(
                    new Translation3d(Units.inchesToMeters(-6.375), Units.inchesToMeters(12),
                            Units.inchesToMeters(8.75)),
                    new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(13),
                            Units.degreesToRadians(135)))));

    // add actual limits
    private final SlewRateLimiter xLimiter1, xLimiter2, xLimiter3, xLimiter4, yLimiter1, yLimiter2, yLimiter3,
            yLimiter4, thetaLimiter1, thetaLimiter2, thetaLimiter3, thetaLimiter4;
    private static final double DEFAULT_BASE_SPEED = 0.3;

    private final LoggedDashboardChooser<Command> autoChooser;

    private final CommandXboxController driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        // change the rate limit values for these when everything else is done

        // xlimiter is used for x and y!!
        xLimiter1 = new SlewRateLimiter(0.3); //l4
        xLimiter2 = new SlewRateLimiter(1.0); //l3
        xLimiter3 = new SlewRateLimiter(2.0); //l2
        xLimiter4 = new SlewRateLimiter(4); //ll1

        yLimiter1 = new SlewRateLimiter(0.3); //l4
        yLimiter2 = new SlewRateLimiter(1.0);
        yLimiter3 = new SlewRateLimiter(2.0);
        yLimiter4 = new SlewRateLimiter(4);

        thetaLimiter1 = new SlewRateLimiter(0.5); //l4
        thetaLimiter2 = new SlewRateLimiter(3.0);
        thetaLimiter3 = new SlewRateLimiter(3.0);
        thetaLimiter4 = new SlewRateLimiter(3.0);

        NamedCommands.registerCommand("Guarentee Stop", realDrivetrainStop());
        NamedCommands.registerCommand("Acquire", coralManipulator.runMotorCommand(.7)
                .alongWith(Commands.waitUntil(
                        coralManipulator.hasGamePieceSupplier()))
                .andThen(coralManipulator.runMotorCommand(0)));
        NamedCommands.registerCommand("Score Outfeed",
                Commands.waitUntil(armistice.armAndElevatorAtTarget()).andThen(Commands.waitSeconds(0.5))
                        .andThen(coralManipulator.runMotorCommand(-.8).alongWith(Commands.waitSeconds(1))
                                .andThen(coralManipulator.runMotorCommand(0))));

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

    public final void simCallback() {
        RobotSim.update(armistice.getSimData());

        RobotSim.logMechanism();
    }

    public void periodicLL4IMU(boolean on) {
        ll4.setIMUInternal(on);
    }

    public void logLLPoses() {
        VisionUtil.logPoses(drive);
    }

    public void seedll4IMU() {
        ll4.seedLLSolverYaw(drive.getPose().getRotation().getDegrees());
    }

    public void disableArmistice() {
        armistice.orbitalStrike();
    }

    private void configureBindings() {
        // not working
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> scaleDriverController(() -> driverController.getLeftY(), LimiterState.X),
                        () -> scaleDriverController(() -> driverController.getLeftX(), LimiterState.Y),
                        () -> scaleDriverController(() -> -driverController.getRightX(),
                                LimiterState.THETA)));

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

        driverController.leftTrigger().and(driverController.back().negate()).onTrue(algae.runMotorCommand(0.5))
                .onFalse(algae.runMotorCommand(0));
        driverController.leftBumper().and(driverController.back().negate()).onTrue(algae.runMotorCommand(-0.8))
                .onFalse(algae.runMotorCommand(0));
        driverController.leftTrigger().and(driverController.back()).onTrue(coralManipulator.runMotorCommand(0.7))
                .onFalse(coralManipulator.runMotorCommand(0));
        driverController.leftBumper().and(driverController.back()).onTrue(coralManipulator.runMotorCommand(-0.8))
                .onFalse(coralManipulator.runMotorCommand(0));

        // //Nudges
        driverController.povUp().onTrue(armistice.nudgeCommand(1, 0));
        driverController.povDown().onTrue(armistice.nudgeCommand(-1, 0));
        driverController.povLeft().onTrue(armistice.nudgeCommand(0, 0.05));
        driverController.povRight().onTrue(armistice.nudgeCommand(0, -0.05));

        // Characterization
        // driverController.povRight().onTrue(armistice.runArmVoltageForChar()).onFalse(armistice.stopArm());
        // driverController.rightBumper().onTrue(armistice.deltaArmCharVolts(0.05));
        // driverController.leftBumper().onTrue(armistice.deltaArmCharVolts(-0.05));

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
        return drive.runOnce(drive::stop);
    }

    public double chooseXLimiter(double input) {
        if (armistice.getElevatorPosition() > 40) {//40
            return xLimiter1.calculate(input);
        } else if (armistice.getElevatorPosition() > 30 && armistice.getElevatorPosition() < 40) {//30
            return xLimiter2.calculate(input);
        } else if (armistice.getElevatorPosition() > 8 && armistice.getElevatorPosition() < 30) {//15
            return xLimiter3.calculate(input);
        } else {//3
            return xLimiter4.calculate(input);
        }
    }

    public double chooseYLimiter(double input) {
        if (armistice.getElevatorPosition() >= 40) {
            return yLimiter1.calculate(input);
        } else if (armistice.getElevatorPosition() > 30 && armistice.getElevatorPosition() < 40) {
            return yLimiter2.calculate(input);
        } else if (armistice.getElevatorPosition() >= 8 && armistice.getElevatorPosition() <= 30) {
            return yLimiter3.calculate(input);
        } else {
            return yLimiter4.calculate(input);
        }
    }

    public double chooseThetaLimiter(double input) {
        if (armistice.getElevatorPosition() > 40) {
            return thetaLimiter1.calculate(input);
        } else if (armistice.getElevatorPosition() > 30 && armistice.getElevatorPosition() < 40) {
            return thetaLimiter2.calculate(input);
        } else if (armistice.getElevatorPosition() > 20 && armistice.getElevatorPosition() < 30) {
            return thetaLimiter3.calculate(input);
        } else {
            return thetaLimiter4.calculate(input);
        }
    }

    // enum

    private double scaleDriverController(DoubleSupplier controllerInput, LimiterState type) {
        double input = controllerInput.getAsDouble() * (DEFAULT_BASE_SPEED
                + (driverController.getRightTriggerAxis() * (1 - DEFAULT_BASE_SPEED)));

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
