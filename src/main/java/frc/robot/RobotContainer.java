// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import javax.sql.rowset.spi.XmlWriter;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Armistice.ArmisticePositions;
import frc.robot.Constants.OperatorConstants;
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
    // private final Limelight ll4ii = new Limelight(new LimelightIO("limelight-fourii", true, null));
    // private final Limelight ll4iii = new Limelight(new LimelightIO("limelight-fouriii", true, Optional.empty()));

    private static final double SLOW_SPEED = 0.07;
    private static final double DEFAULT_BASE_SPEED = 0.3;
    private double speedySpeed = DEFAULT_BASE_SPEED;
    private boolean[] futuresOn = new boolean[6];

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
    private final CommandXboxController emergencyController = new CommandXboxController(OperatorConstants.kEmergencyControllerPort);

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

    public final void simCallback() {
        RobotSim.update(armistice.getSimData());

        RobotSim.logMechanism();
    }

    public void disableArmistice() {
        armistice.orbitalStrike();
    }

    private void configureBindings() {
        // driverController.a().and(driverController.b()).onTrue(Commands.runOnce(this::disableArmistice));
    
        driverController.povUp().onTrue(armistice.nudgeCommand(0.5, 0.0));
        driverController.povDown().onTrue(armistice.nudgeCommand(-0.5, 0.0));
        driverController.povRight().onTrue(armistice.nudgeCommand(0, 0.1));
        driverController.povLeft().onTrue(armistice.nudgeCommand(0, -0.1));

        driverController.leftTrigger().onTrue(coral.runMotorCommand(0.45))
                .onFalse(coral.runMotorCommand(0));
        driverController.leftBumper().onTrue(coral.runMotorCommand(-0.40))
                .onFalse(coral.runMotorCommand(0));

        driverController.rightBumper().onTrue(Commands.runOnce(() -> speedySpeed = SLOW_SPEED))
                .onFalse(Commands.runOnce(() -> speedySpeed = DEFAULT_BASE_SPEED));
            
        // Reset gyro to 0° when start button is pressed
        driverController.start().onTrue(
                Commands.runOnce(
                        () -> drive.setPose(
                                new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                        drive)
                        .ignoringDisable(true));

        // driverController.a().onTrue(armistice.runArmVoltageForChar());
        // driverController.rightBumper().onTrue(armistice.deltaArmCharVolts(0.1));
        // driverController.leftBumper().onTrue(armistice.deltaArmCharVolts(-0.1));
        // driverController.x().and(driverController.rightBumper()).whileTrue(armistice.sysIDCommandElevator(()
        // -> true, () -> Direction.kForward));
        // driverController.x().and(driverController.leftBumper()).whileTrue(armistice.sysIDCommandElevator(()
        // -> true, () -> Direction.kReverse));
        // driverController.y().and(driverController.rightBumper()).whileTrue(armistice.sysIDCommandElevator(()
        // -> false, () -> Direction.kForward));
        // driverController.y().and(driverController.leftBumper()).whileTrue(armistice.sysIDCommandElevator(()
        // -> false, () -> Direction.kReverse));

        // =================== //
        /* OPERATOR CONTROLLER */
        // =================== //

        // Algae
        operatorController.leftTrigger().onTrue(algae.runMotorCommand(.7)).onFalse(algae.runMotorCommand(0));
        operatorController.rightTrigger().onTrue(algae.runMotorCommand(-.7)).onFalse(algae.runMotorCommand(0));

        // Elevator
        operatorController.rightBumper().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.STOW));
        operatorController.x().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.ACQUIRE));
        operatorController.y().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.L4));
        operatorController.b().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.L3));
        operatorController.a().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.L2));

        // Reef Algae, Barge, Lollipop Misc.
        operatorController.povUp().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.BARGE_REAL));
        operatorController.povDown().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.LOLLIPOP_ACQUIRE));
        operatorController.povLeft().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.ALGAE_AQUIRE_L2));
        operatorController.povRight().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.ALGAE_AQUIRE_L3));
        // cool reefstate thingy
        operatorController.rightBumper().onTrue(armistice.changeFutureArmisticePosition(1));
        operatorController.rightBumper().onTrue(armistice.changeFutureArmisticePosition(-1));
        operatorController.back().onTrue(armistice.runToFutureArmisticePositionCommand());
        // ==================== //
        /* EMERGENCY CONTROLLER */
        // ==================== //
        emergencyController.rightBumper().onTrue(climber.runVbusCommand(0.2)).onFalse(climber.runVbusCommand(0));
        emergencyController.leftBumper().onTrue(climber.runVbusCommand(-0.2)).onFalse(climber.runVbusCommand(0));

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

    private double scaleDriverController(DoubleSupplier controllerInput, LimiterState type) {
        double input = controllerInput.getAsDouble() * ((speedySpeed)
                + (driverController.getRightTriggerAxis() * (1 - speedySpeed)));
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
