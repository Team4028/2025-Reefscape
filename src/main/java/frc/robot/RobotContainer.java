// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

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
import frc.robot.util.RobotSim;

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
    private static final double DEFAULT_BASE_SPEED = 0.3;
    private final LoggedDashboardChooser<Command> autonChooser = new LoggedDashboardChooser<>("Auton Chooser");
    private final Drive drive = RobotSim.driveSimSwitch(new GyroIOPigeon2(), new ModuleIO[] {
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight)
    });

    // add actual limits
    private final SlewRateLimiter xLimiterL4, xLimiterL3, xLimiterL2, xLimiterL1, yLimiterL4, yLimiterL3, yLimiterL2,
            yLimiterL1, thetaLimiterL4, thetaLimiterL3, thetaLimiterL2, thetaLimiterL1;

    private final CommandXboxController driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        xLimiterL4 = new SlewRateLimiter(0.3);
        xLimiterL3 = new SlewRateLimiter(1.0);
        xLimiterL2 = new SlewRateLimiter(2.0);
        xLimiterL1 = new SlewRateLimiter(4);

        yLimiterL4 = new SlewRateLimiter(0.3);
        yLimiterL3 = new SlewRateLimiter(1.0);
        yLimiterL2 = new SlewRateLimiter(2.0);
        yLimiterL1 = new SlewRateLimiter(4);

        thetaLimiterL4 = new SlewRateLimiter(0.5);
        thetaLimiterL3 = new SlewRateLimiter(3.0);
        thetaLimiterL2 = new SlewRateLimiter(3.0);
        thetaLimiterL1 = new SlewRateLimiter(3.0);

        autonChooser.addDefaultOption("Char drivetrain", DriveCommands.feedforwardCharacterization(drive));
        // Set up SysId routines
        configureBindings();
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

        // // A - Elevator
        // driverController.a().and(driverController.rightBumper())
        // .onTrue(Commands.runOnce(() -> armistice.runElevatorVbus(.2),
        // armistice.getSubsystems()))
        // .onFalse(Commands.runOnce(() -> armistice.runElevatorVbus(0),
        // armistice.getSubsystems()));
        // driverController.a().and(driverController.leftBumper())
        // .onTrue(Commands.runOnce(() -> armistice.runElevatorVbus(-.2),
        // armistice.getSubsystems()))
        // .onFalse(Commands.runOnce(() -> armistice.runElevatorVbus(0),
        // armistice.getSubsystems()));
        // // B - Arm
        // driverController.b().and(driverController.rightBumper()).onTrue(Commands.runOnce(()
        // -> armistice.runArmVbus(.2),
        // armistice.getSubsystems()))
        // .onFalse(Commands.runOnce(() -> armistice.runArmVbus(0),
        // armistice.getSubsystems()));
        // driverController.b().and(driverController.leftBumper()).onTrue(Commands.runOnce(()
        // -> armistice.runArmVbus(-.2),
        // armistice.getSubsystems()))
        // .onFalse(Commands.runOnce(() -> armistice.runArmVbus(0),
        // armistice.getSubsystems()));
        driverController.rightBumper().onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.STOW));
        driverController.y().and(driverController.back().negate())
                .onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.ACQUIRE));
        driverController.b().and(driverController.back().negate())
                .onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.L2));
        driverController.x().and(driverController.back().negate())
                .onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.L3));
        driverController.a().and(driverController.back().negate())
                .onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.L4));
        driverController.y().and(driverController.back())
                .onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.LOLLIPOP_ACQUIRE));
        driverController.b().and(driverController.back())
                .onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.ALGAE_AQUIRE_L2));
        driverController.x().and(driverController.back())
                .onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.ALGAE_AQUIRE_L3));
        driverController.a().and(driverController.back())
                .onTrue(armistice.runToPositionCommand(() -> ArmisticePositions.BARGE_REAL));
        driverController.povUp().onTrue(armistice.nudgeCommand(0.5, 0.0));
        driverController.povDown().onTrue(armistice.nudgeCommand(-0.5, 0.0));
        driverController.povRight().onTrue(armistice.nudgeCommand(0, 0.1));
        driverController.povLeft().onTrue(armistice.nudgeCommand(0, -0.1));
        driverController.back().and(driverController.leftTrigger()).onTrue(algae.runMotorCommand(0.5))
                .onFalse(algae.runMotorCommand(0));
        driverController.back().and(driverController.leftBumper()).onTrue(algae.runMotorCommand(-0.9))
                .onFalse(algae.runMotorCommand(0));
        driverController.leftTrigger().and(driverController.back().negate()).onTrue(coral.runMotorCommand(0.45))
                .onFalse(coral.runMotorCommand(0));
        driverController.leftBumper().and(driverController.back().negate()).onTrue(coral.runMotorCommand(-0.55))
                .onFalse(coral.runMotorCommand(0));
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> scaleDriverController(() -> driverController.getLeftY(), LimiterState.X),
                        () -> scaleDriverController(() -> driverController.getLeftX(), LimiterState.Y),
                        () -> scaleDriverController(() -> -driverController.getRightX(),
                                LimiterState.THETA)));

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
        // X - Coral
        // driverController.x().and(driverController.rightBumper()).onTrue(coral.runMotorCommand(0.2)).onFalse(coral.runMotorCommand(0));
        // driverController.x().and(driverController.leftBumper()).onTrue(coral.runMotorCommand(-0.2)).onFalse(coral.runMotorCommand(0));
        // // Y - Algae
        // driverController.y().and(driverController.rightBumper()).onTrue(algae.runMotorCommand(0.2)).onFalse(algae.runMotorCommand(0));
        // driverController.y().and(driverController.leftBumper()).onTrue(algae.runMotorCommand(-0.2)).onFalse(algae.runMotorCommand(0));

        // Back - Climber
        // driverController.back().and(driverController.rightBumper()).onTrue(climber.runVbusCommand(0.2))
        // .onFalse(climber.runVbusCommand(0));
        // driverController.back().and(driverController.leftBumper()).onTrue(climber.runVbusCommand(-0.2))
        // .onFalse(climber.runVbusCommand(0));
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
        } else if (a >= 30 && a < 40) {
            return xLimiterL3.calculate(input);
        } else if (a >= 8 && a <= 30) {
            return xLimiterL2.calculate(input);
        } else {
            return xLimiterL1.calculate(input);
        }
    }

    public double chooseYLimiter(double input) {
        var a = armistice.getElevatorPosition();
        if (a > 40) {
            return yLimiterL4.calculate(input);
        } else if (a >= 30 && a < 40) {
            return yLimiterL3.calculate(input);
        } else if (a >= 8 && a <= 30) {
            return yLimiterL2.calculate(input);
        } else {
            return yLimiterL1.calculate(input);
        }
    }
    
    public double chooseThetaLimiter(double input) {
        var a = armistice.getElevatorPosition();
        if (a > 40) {
            return thetaLimiterL4.calculate(input);
        } else if (a >= 30 && a < 40) {
            return thetaLimiterL3.calculate(input);
        } else if (a >= 8 && a <= 30) {
            return thetaLimiterL2.calculate(input);
        } else {
            return thetaLimiterL1.calculate(input);
        }
    }

    private double scaleDriverController(DoubleSupplier controllerInput, LimiterState type) {
        double input = controllerInput.getAsDouble() * (DEFAULT_BASE_SPEED)
                + (driverController.getRightTriggerAxis() * (1 - DEFAULT_BASE_SPEED));
        switch (type) {
            case X:
                return chooseXLimiter(input);
            case Y:
                return chooseYLimiter(input);
            case THETA:
                return chooseXLimiter(input);
            default:
                return 0.0;
        }
       
    }

}
