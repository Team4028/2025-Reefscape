// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.SudoSubsystem;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;

    public Robot() {
        robotContainer = new RobotContainer();
        SignalLogger.setPath("/media/sda1/ctre");
        Logger.recordMetadata("ProjectName", "BeakSquad4028");
        Logger.recordMetadata("TimeStamp", ZonedDateTime.now().format(DateTimeFormatter.ISO_DATE_TIME));

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
    public void robotPeriodic() {
        if (DriverStation.isEnabled()) {
            SudoSubsystem.periodicAll();
        }
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        SignalLogger.stop();
        robotContainer.disableArmistice();
    }
    
    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }

        robotContainer.resetArmPid();
    }

    @Override
    public void autonomousPeriodic() {
    }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotContainer.resetArmPid();

    // SignalLogger.start();
  }

    @Override
    public void teleopPeriodic() {
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
