package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Armistice;
import frc.robot.Armistice.ArmisticePositions;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.stick.WhipStick;
import lombok.experimental.UtilityClass;

@UtilityClass
public class MagicSequencing {

    public static final Command magicScoreScore(Drive drive, Armistice armistice, WhipStick coral,
            Supplier<Pose2d> pReefPose, Supplier<Pose2d> reefPose, Supplier<Pose2d> tReefPose,
            Supplier<ArmisticePositions> scorePos) {
        return Commands.runOnce(() -> armistice.setSafety(false))
                .andThen(drive.translateToPositionWithPID(pReefPose.get()).until(armistice.armAndElevatorAtTarget())
                        .alongWith(armistice.runToPositionNoWait(scorePos.get(), drive.closestReefName(),
                                drive.getReefTargetIsRight()))
                        .andThen(drive.translateToPositionWithPID(reefPose.get()).until(drive.translatePidInPosition()))
                        .andThen(armistice.runToPositionCommand(scorePos.get().getSCPose(), drive.closestReefName(),
                                drive.getReefTargetIsRight()))
                        .andThen(Commands.waitUntil(armistice.armAndElevatorAtTarget()))
                        .andThen(Commands.waitUntil(armistice.armAndElevatorAtTarget()))
                        .andThen(drive.translateToPositionWithPID(tReefPose.get())
                                .until(drive.translatePidInPosition())))
                .andThen(Commands.runOnce(() -> armistice.setSafety(true)));
    }

    public static final Command magicShank(Armistice armistice, WhipStick coral, Command waitForDown,
            Command waitForUp) {
        return armistice
                .runToPositionNoWait(ArmisticePositions.CLEAN)
                .andThen(waitForDown)
                .andThen(armistice.runToPositionNoWait(ArmisticePositions.SHANK))
                .andThen(waitForUp)
                .andThen(armistice.runToPositionNoWait(ArmisticePositions.CLEAN));
    }

    public static final Command magicScoreReef(Drive drive, Armistice armistice, WhipStick coral,
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

    public static final Command magicAquireReefAlgae(Drive drive, Armistice armistice, WhipStick algae,
            Supplier<Pose2d> reefPostiion, Supplier<ArmisticePositions> aquirePosition) {
        return drive.pathfindToPose(reefPostiion.get())
                .alongWith(Commands.waitUntil(drive.readyForArm())
                        .andThen(armistice.runToPositionNoWait(aquirePosition.get()))
                        .alongWith(algae.runMotorCommand(0.7)))
                .andThen(algae.runMotorCommand(0.7).repeatedly()
                        .until(algae.hasGamePieceSupplier())
                        .withTimeout(1)
                        .andThen(algae.runMotorCommand(0))
                        .raceWith(drive.translateToPositionWithPID(reefPostiion.get())));
    }

    public static final Command magicScoreNoScoreReef(Drive drive, Armistice armistice, WhipStick coral,
            Supplier<Pose2d> reefPosition, Supplier<ArmisticePositions> scorePosition) {
        return drive.pathfindToPose(reefPosition.get())
                .alongWith(Commands.waitUntil(drive.readyForArm())
                        .andThen(armistice.runToPositionNoWait(
                                (scorePosition.get() == ArmisticePositions.BARGE ? ArmisticePositions.Cora_L4
                                        : scorePosition.get()))))
                .andThen(drive.translateToPositionWithPID(reefPosition.get()).until(drive.translatePidInPosition()));
    }

    public static final Command magicScoreNoScoreReefOnlyPID(Drive drive, Armistice armistice, WhipStick coral,
            Supplier<Pose2d> reefPosition, Supplier<ArmisticePositions> scorePosition) {
        return drive.translateToPositionWithPID(reefPosition.get())
                .alongWith(Commands.waitUntil(drive.readyForArm())
                        .andThen(armistice.runToPositionNoWait(
                                (scorePosition.get() == ArmisticePositions.BARGE ? ArmisticePositions.Cora_L4
                                        : scorePosition.get()))));
    }

    public static final Command magicGetAlgaeOnlyPID(Drive drive, Armistice armistice, WhipStick algae,
            Supplier<Pose2d> reefPosition, Supplier<ArmisticePositions> acquirePosition) {
        return drive.translateToPositionWithPID(reefPosition.get())
                .alongWith(armistice.runToPositionNoWait(acquirePosition.get()))
                .raceWith(algae.runMotorCommand(0.7).repeatedly()
                        .until(algae.hasGamePieceSupplier()).withTimeout(2).andThen(algae.runMotorCommand(0)));
    }

    public static final Command magicScoreSuperCycleLOther(Drive drive, Armistice armistice, WhipStick coral,
            Supplier<Pose2d> reefPosition, Supplier<Pose2d> algaePosition,
            Supplier<ArmisticePositions> scorePosition, Supplier<ArmisticePositions> acquirePosition) {
        if (scorePosition.get() == ArmisticePositions.Cora_L4)
            return Commands.none();
        return magicGetAlgaeOnlyPID(drive, armistice, coral, algaePosition, acquirePosition).andThen(
                drive.translateToPositionWithPID(algaePosition.get().transformBy(new Transform2d(
                        new Translation2d(-.4, 0).rotateBy(Constants.SCORING_SIDE_FROM_FRONT_ROT), Rotation2d.kZero)))
                        .until(drive.translatePidInPositionJankier()))
                .andThen(magicScoreNoScoreReefOnlyPID(drive, armistice, coral, reefPosition, scorePosition)
                        .until(drive.translatePidInPosition()));
        // .andThen(Commands.waitUntil(armistice.armAndElevatorAtTarget()))
        // .andThen(Commands.waitSeconds(0.2)
        // .andThen(coral.runMotorCommand(-.4).repeatedly().withTimeout(.3))
        // .andThen(coral.runMotorCommand(0))));
    }

    public static final Command magicScoreSuperCycleL4(Drive drive, Armistice armistice, WhipStick coral,
            Supplier<Pose2d> reefPosition, Supplier<Pose2d> algaePosition,
            Supplier<ArmisticePositions> scorePosition, Supplier<ArmisticePositions> acquirePosition) {
        return magicScoreNoScoreReefOnlyPID(drive, armistice, coral, reefPosition, scorePosition)
                .until(drive.translatePidInPosition())
                .andThen(Commands.waitUntil(armistice.armAndElevatorAtTarget()))
                .andThen(Commands.waitSeconds(0.2).andThen(coral.runMotorCommand(-.8).repeatedly().withTimeout(0.3))
                        .andThen(coral.runMotorCommand(0)))
                .andThen(armistice.runToPositionNoWait(ArmisticePositions.STOW))
                .andThen(drive.translateToPositionWithPID(reefPosition.get().transformBy(new Transform2d(
                        new Translation2d(-.3, 0).rotateBy(Constants.SCORING_SIDE_FROM_FRONT_ROT), Rotation2d.kZero))))
                .until(drive.translatePidInPosition())
                .andThen(magicGetAlgaeOnlyPID(drive, armistice, coral, algaePosition, acquirePosition));
    }

    public static final Command magicBackUpAndMagicAlgaeL4(Drive drive, Armistice armistice, WhipStick algae,
            Supplier<Pose2d> reefPosition, Supplier<Pose2d> algaePosition,
            Supplier<ArmisticePositions> acquirePosition) {
        return armistice.runToPositionNoWait(acquirePosition.get())
                .andThen(drive.translateToPositionWithPID(reefPosition.get().transformBy(new Transform2d(
                        new Translation2d(-.3, 0).rotateBy(Constants.SCORING_SIDE_FROM_FRONT_ROT), Rotation2d.kZero)))
                        .until(drive.translatePidInPositionJankier()))
                .andThen(magicGetAlgaeOnlyPID(drive, armistice, algae, algaePosition, acquirePosition));
    }
}