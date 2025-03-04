package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Armistice;
import frc.robot.Armistice.ArmisticePositions;
import frc.robot.subsystems.algae.AlgaeManipulator;
import frc.robot.subsystems.coral.CoralManipulator;
import frc.robot.subsystems.drive.Drive;

public class AutoSequencing {
    public static final Command autoScoreReef(Drive drive, Armistice armistice, CoralManipulator coral,
            Supplier<Pose2d> reefPosition, Supplier<ArmisticePositions> scorePosition) {
        return drive.pathfindToPose(reefPosition.get())
                .alongWith(Commands.waitUntil(drive.readyForArm())
                        .andThen(armistice.runToPositionNoWait(
                                (scorePosition.get() == ArmisticePositions.BARGE ? ArmisticePositions.Cora_L4
                                        : scorePosition.get()))))
                .andThen(Commands.waitUntil(armistice.armAndElevatorAtTarget()).andThen(Commands.waitSeconds(.3))
                        .andThen(coral.runMotorCommand(-0.8))
                        .andThen(Commands.waitSeconds(0.3))
                        .andThen(coral.runMotorCommand(0)
                                .alongWith(armistice.runToPositionCommand(ArmisticePositions.STOW)))
                        .raceWith(drive.translateToPositionWithPID(reefPosition.get())));
    }

    public static final Command autoAquireReefAlgae(Drive drive, Armistice armistice, AlgaeManipulator algae,
            Supplier<Pose2d> reefPostiion, Supplier<ArmisticePositions> aquirePosition) {
        return drive.pathfindToPose(reefPostiion.get())
                .alongWith(Commands.waitUntil(drive.readyForArm())
                        .andThen(armistice.runToPositionNoWait(aquirePosition.get())).alongWith(algae.runMotorCommand(0.7)))
                .andThen(algae.runMotorCommand(0.7).repeatedly()
                        .until(algae.hasGamePieceSupplier())
                        .withTimeout(1)
                        .andThen(algae.runMotorCommand(0))
                        .raceWith(drive.translateToPositionWithPID(reefPostiion.get())));
    }

    public static final Command autoScoreReefNoShoot(Drive drive, Armistice armistice, CoralManipulator coral,
            Supplier<Pose2d> reefPosition, Supplier<ArmisticePositions> scorePosition) {
        return drive.pathfindToPose(reefPosition.get())
                .alongWith(Commands.waitUntil(drive.readyForArm())
                        .andThen(armistice.runToPositionNoWait(
                                (scorePosition.get() == ArmisticePositions.BARGE ? ArmisticePositions.Cora_L4
                                        : scorePosition.get()))))
                .andThen(drive.translateToPositionWithPID(reefPosition.get()).until(drive.translatePidInPosition()));
    }
}