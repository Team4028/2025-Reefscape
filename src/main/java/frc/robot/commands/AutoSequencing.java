package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Armistice;
import frc.robot.Armistice.ArmisticePositions;
import frc.robot.subsystems.coral.CoralManipulator;
import frc.robot.subsystems.drive.Drive;

public class AutoSequencing {
    public static final Command autoScoreReef(Drive drive, Armistice armistice, CoralManipulator coral,
            Supplier<Pose2d> reefPosition, Supplier<ArmisticePositions> scorePosition) {
        return drive.pathfindToPose(reefPosition.get()).andThen(armistice.runToPositionCommand(scorePosition.get())
                .andThen(Commands.waitSeconds(0.5)).andThen(coral.runMotorCommand(-0.8))
                .andThen(Commands.waitSeconds(0.25)).andThen(coral.runMotorCommand(0))
                .alongWith(drive.translateToPositionWithPID(reefPosition.get())));
    }
}
