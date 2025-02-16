// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Armistice.ArmisticePositions;
import frc.robot.Constants.OperatorConstants;

import frc.robot.util.RobotSim;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    public enum LimiterState {
        X,
        Y,
        THETA
    }

    private final Armistice armistice = new Armistice();

    // add actual limits
    private final SlewRateLimiter xLimiter1, xLimiter2, xLimiter3, xLimiter4, yLimiter1, yLimiter2, yLimiter3,
            yLimiter4, thetaLimiter1, thetaLimiter2, thetaLimiter3, thetaLimiter4;

    private final LoggedDashboardChooser<Command> autoChooser;

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

        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
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
        // not working

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
    }

    public void resetArmPid() {
        armistice.resetArmPid();
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
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
