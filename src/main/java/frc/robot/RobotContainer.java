// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Armistice.ArmisticePositions;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.algae.AlgaeManipulator;
import frc.robot.subsystems.algae.AlgaeManipulatorIOTalonFX;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.coral.CoralManipulator;
import frc.robot.subsystems.coral.CoralManipulatorIOTalonFX;
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

    // add actual limits
    private final SlewRateLimiter xLimiter1, xLimiter2, xLimiter3, xLimiter4, yLimiter1, yLimiter2, yLimiter3,
            yLimiter4, thetaLimiter1, thetaLimiter2, thetaLimiter3, thetaLimiter4;



    private final CommandXboxController driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        // change the rate limit values for these when everything else is done

        // xlimiter is used for x and y!!
        xLimiter1 = new SlewRateLimiter(0.3); // l4
        xLimiter2 = new SlewRateLimiter(1.0); // l3
        xLimiter3 = new SlewRateLimiter(2.0); // l2
        xLimiter4 = new SlewRateLimiter(4); // ll1

        yLimiter1 = new SlewRateLimiter(0.3); // l4
        yLimiter2 = new SlewRateLimiter(1.0);
        yLimiter3 = new SlewRateLimiter(2.0);
        yLimiter4 = new SlewRateLimiter(4);

        thetaLimiter1 = new SlewRateLimiter(0.5); // l4
        thetaLimiter2 = new SlewRateLimiter(3.0);
        thetaLimiter3 = new SlewRateLimiter(3.0);
        thetaLimiter4 = new SlewRateLimiter(3.0);

      
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

        driverController.a().and(driverController.b()).onTrue(Commands.runOnce(this::disableArmistice));

        // A - Elevator
        driverController.a().and(driverController.rightBumper().onTrue(Commands.runOnce(() -> armistice.runElevatorVbus(.95), armistice.getSubsystems())).onFalse(Commands.runOnce(() -> armistice.runElevatorVbus(0), armistice.getSubsystems())));
        driverController.a().and(driverController.leftBumper().onTrue(Commands.runOnce(() -> armistice.runElevatorVbus(-.25), armistice.getSubsystems())).onFalse(Commands.runOnce(() -> armistice.runElevatorVbus(0), armistice.getSubsystems())));
        // B - Arm
        driverController.b().and(driverController.rightBumper().onTrue(Commands.runOnce(() -> armistice.runArmVbus(.25), armistice.getSubsystems())).onFalse(Commands.runOnce(() -> armistice.runArmVbus(0), armistice.getSubsystems())));
        driverController.b().and(driverController.leftBumper().onTrue(Commands.runOnce(() -> armistice.runArmVbus(-.25), armistice.getSubsystems())).onFalse(Commands.runOnce(() -> armistice.runArmVbus(0), armistice.getSubsystems())));
        // X - Coral
        driverController.x().and(driverController.rightBumper().onTrue(coral.runMotorCommand(0.2)).onFalse(coral.runMotorCommand(0)));
        driverController.x().and(driverController.leftBumper().onTrue(coral.runMotorCommand(-0.2)).onFalse(coral.runMotorCommand(0)));
        // Y - Algae
        driverController.y().and(driverController.rightBumper().onTrue(algae.runMotorCommand(0.2)).onFalse(algae.runMotorCommand(0)));
        driverController.y().and(driverController.leftBumper().onTrue(algae.runMotorCommand(-0.2)).onFalse(algae.runMotorCommand(0)));
        // Back - Climber
        driverController.back().and(driverController.rightBumper().onTrue(climber.runVbusCommand(0.2)).onFalse(climber.runVbusCommand(0)));
        driverController.back().and(driverController.leftBumper().onTrue(climber.runVbusCommand(-0.2)).onFalse(climber.runVbusCommand(0)));
    }

    public void resetArmPid() {
        armistice.resetArmPid();
    }

    public double chooseXLimiter(double input) {
        if (armistice.getElevatorPosition() > 40) {// 40
            return xLimiter1.calculate(input);
        } else if (armistice.getElevatorPosition() > 30 && armistice.getElevatorPosition() < 40) {// 30
            return xLimiter2.calculate(input);
        } else if (armistice.getElevatorPosition() > 8 && armistice.getElevatorPosition() < 30) {// 15
            return xLimiter3.calculate(input);
        } else {// 3
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

}
