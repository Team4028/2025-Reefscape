// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Elastic;
import frc.robot.util.SudoSubsystem;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;

    public Robot() {
        robotContainer = new RobotContainer();
        Logger.recordMetadata("ProjectName", "2025-Reefscape");
        Logger.recordMetadata("TimeStamp", ZonedDateTime.now().format(DateTimeFormatter.ISO_DATE_TIME));
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All Changes Committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommited Changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        switch (Constants.currentMode) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIM:
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }
        Logger.start();
    }

    @Override
    public void robotInit() {
        FollowPathCommand.warmupCommand().schedule();
    }

    @Override
    public void robotPeriodic() {
        if (robotContainer.getDeadmanGood()) {
            SudoSubsystem.robotPeriodicAll();
            if (DriverStation.isEnabled()) {
                SudoSubsystem.periodicAll();
            }
            CommandScheduler.getInstance().run();
        } else {
            robotContainer.drivetrainStop();
        }
        robotContainer.logLLPoses();
        robotContainer.seedll4IMU();
        robotContainer.updateArmisticeAutoAlgae();
    }

    @Override
    public void disabledInit() {
        robotContainer.disableArmisticeArm();
    }

    @Override
    public void disabledPeriodic() {
        robotContainer.periodicLL4IMU(false);
    }

    @Override
    public void autonomousInit() {
        robotContainer.enableArmisticeArm();
        robotContainer.startAutonTimer();
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
        robotContainer.periodicLL4IMU(true);
    }

    @Override
    public void autonomousPeriodic() {
        robotContainer.addMeasurements();
        // robotContainer.turnOnIfGood();
    }

    @Override
    public void teleopInit() {
        robotContainer.enableArmisticeArm();
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        robotContainer.periodicLL4IMU(true);
        Elastic.selectTab("Teleoperated");
    }

    @Override
    public void teleopPeriodic() {
        robotContainer.addMeasurements();
        // robotContainer.turnOnIfGood();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
        robotContainer.simCallback();
        Logger.recordOutput("Robot/BatteryVoltage", RobotController.getBatteryVoltage());
    }
}
