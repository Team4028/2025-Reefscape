// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSparkEncoderTalonFX;
import frc.robot.subsystems.coral.CoralManipulator;
import frc.robot.subsystems.coral.CoralManipulatorIOTalonSRX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.util.RobotSim;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // TODO: log motor pos + vel + temp in all subsystems (& also test) (& also
    // TODO: replace or augment stateTracker with annotation states)
    // The robot's subsystems and commands are defined here...
    private final Elevator elevator = new Elevator(RobotSim.elevatorSimSwitch(new ElevatorIOTalonFX()));
    private final Arm arm = new Arm(RobotSim.armSimSwitch(new ArmIOSparkEncoderTalonFX()), elevator);
    private final CoralManipulator coralManipulator = new CoralManipulator(
            RobotSim.coralManipulatorSimSwitch(new CoralManipulatorIOTalonSRX()));

    private final Drive drive = RobotSim.driveSimSwitch(new GyroIOPigeon2(),
            new ModuleIO[] { new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight), new ModuleIOTalonFX(TunerConstants.BackLeft),
                    new ModuleIOTalonFX(TunerConstants.BackRight) });

    private final SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

    private final LoggedDashboardChooser<Command> autoChooser;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    @SuppressWarnings("unused")
    private final CommandXboxController driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        xLimiter = new SlewRateLimiter(4);
        yLimiter = new SlewRateLimiter(4);
        thetaLimiter = new SlewRateLimiter(4);
        NamedCommands.registerCommand("L4 Score",
                runToL4());
        NamedCommands.registerCommand("Stow",
                runToStow());
        NamedCommands.registerCommand("Acquire",
                runAquire().andThen(coralManipulator.runMotorCommand(.7).alongWith(Commands.waitSeconds(1))
                        .andThen(coralManipulator.runMotorCommand(0))));
        NamedCommands.registerCommand("L3 Score",
                runToL3().andThen(Commands.waitUntil(arm.atTargetPosition()))
                        .andThen(coralManipulator.runMotorCommand(-.8)
                                .alongWith(Commands.waitSeconds(1))
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

    public static final double notNaN(double x) {
        return x == Double.NaN ? 0 : x;
    }

    public final void simCallback() {
        RobotSim.update(elevator.getSimPos(), arm.getSimAngle());

        RobotSim.logMechanism();
    }

    private void configureBindings() {
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX()));

        driverController.x().onTrue(runToL4());
        // Run to L3
        driverController.a().onTrue(runToL3());
        // Run to L2
        driverController.b().onTrue(runToL2());
        // Acquire
        driverController.y().onTrue(runAquire());

        driverController.rightBumper().onTrue(runToStow());

        driverController.leftTrigger().onTrue(elevator.runMotorsCommand(0.8)).onFalse(elevator.runMotorsCommand(0));
        driverController.leftBumper().onTrue(elevator.runMotorsCommand(-0.8)).onFalse(elevator.runMotorsCommand(0));

        // Reset gyro to 0° when B button is pressed
        driverController
                .start()
                .onTrue(
                        Commands.runOnce(
                                () -> drive.setPose(
                                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                                drive)
                                .ignoringDisable(true));
    }

    public void resetArmPid() {
        arm.pidReset();
    }

    public void runArmOff() {
        arm.stop();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.get();
    }

    public Command runToL4() {
        return elevator.runToPositionCommand(50).alongWith(Commands.waitUntil(elevator.atTargetPosition()))
                .andThen(arm.runToPositionCommand(Units.degreesToRadians(65.)));
    }

    public Command runToL3() {
        return elevator.runToPositionCommand(56).alongWith(Commands.waitUntil(elevator.atTargetPosition()))
                .andThen(arm.runToPositionCommand(Units.degreesToRadians(55)));
    }

    public Command runToL2() {
        return elevator.runToPositionCommand(40.5).alongWith(Commands.waitUntil(elevator.atTargetPosition()))
                .andThen(arm.runToPositionCommand(Units.degreesToRadians(55)));
    }

    public Command runAquire() {
        return elevator.runToPositionCommand(16).alongWith(Commands.waitUntil(elevator.atTargetPosition()))
                .andThen(arm.runToPositionCommand(Units.degreesToRadians(235)));
    }

    public Command runToStow() {
        return elevator.runToPositionCommand(8).alongWith(Commands.waitUntil(elevator.atTargetPosition()))
                .andThen(arm.runToPositionCommand(Units.degreesToRadians(180)));
    }
}
