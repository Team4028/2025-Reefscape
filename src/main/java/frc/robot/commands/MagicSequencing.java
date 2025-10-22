package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Armistice;
import frc.robot.Armistice.ArmisticePositions;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.stick.WhipStick;
import frc.robot.util.MiscUtils;
import lombok.experimental.ExtensionMethod;
import lombok.experimental.UtilityClass;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

@ExtensionMethod({MiscUtils.class, DriveCommands.class, Commands.class})
@UtilityClass
public class MagicSequencing {
    public static boolean isMagicScoreRunning = false;

    public static Command magicScoreNoBackup(Drive drive, Armistice armistice, WhipStick coral,
                                         Supplier<Pose2d> reefPose, Supplier<ArmisticePositions> scorePos) {
        return Commands.runOnce(() -> isMagicScoreRunning = true)
                .andThen(drive.translateToPositionWithPID(reefPose.get())
                        .raceWith(Commands.defer(
                                                () -> drive.waitForDrivetrainDistance(0.006),
                                                Set.of())
                                        .alongWith(Commands.waitUntil(() -> armistice.getTargetPosition().isSC()
                                                && armistice.armAndElevatorAtTarget().getAsBoolean())),
                                Commands.runOnce(() -> armistice.setSafety(false))
                                        .andThen(Commands.waitUntil(drive.readyForArm())
                                                .andThen(armistice.runToPositionNoWait(scorePos.get().toPipe()))
                                                .andThen(
                                                        Commands.defer(
                                                                        () -> drive.waitForDrivetrainDistance(
                                                                                switch (scorePos.get().getUnPipe()) {
                                                                                    case Cora_L4 -> 0.5;
                                                                                    case Cora_L3 -> 0.3;
                                                                                    default -> 0.2;
                                                                                }),
                                                                        Set.of())
                                                                .andThen(armistice.waitUntilThingsInTolerance(3,
                                                                        0.3))
                                                                .andThen(
                                                                        armistice.runToPositionNoWait(
                                                                                scorePos.get().getSCPose()
                                                                                        .toPipe(),
                                                                                drive.closestReefName(),
                                                                                drive.getReefTargetIsRight()))
                                                                .andThen(Commands.defer(
                                                                        () -> armistice
                                                                                .waitUntilThingsInTolerance(1,
                                                                                        scorePos.get() == ArmisticePositions.Cora_L4
                                                                                                ? 0.3
                                                                                                : 0.1),
                                                                        Set.of())))))
                        .andThen(Commands.defer(
                                () -> armistice.waitUntilThingsInTolerance(1,
                                        scorePos.get() == ArmisticePositions.Cora_L4 ? 0.6 : 0.3),
                                Set.of()))
                        .andThen(drive.runVelocityAngle(() -> 0, () -> -3, drive::getRotation)
                                .alongWith(coral.runMotorCommand(-.3))
                                .withTimeout(0.25)
                                .andThen(coral.runMotorCommand(0))
                                .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW)))
                        .finallyDo(() -> armistice.waitUntilThingsInTolerance(3, Units.degreesToRadians(5))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(!(isMagicScoreRunning = false)))
                                        .onlyIf(() -> !isMagicScoreRunning))
                                .schedule()));
    }

    public static Command magicScoreSafeNoBackup(Drive drive, Armistice armistice, WhipStick coral,
                                                     Supplier<Pose2d> reefPose, Supplier<ArmisticePositions> scorePos) {
        return Commands.runOnce(() -> isMagicScoreRunning = true)
                .andThen(drive.translateToPositionWithPID(reefPose.get())
                        .raceWith(Commands.defer(
                                                () -> drive.waitForDrivetrainDistance(0.006),
                                                Set.of())
                                        .alongWith(Commands.waitUntil(() -> armistice.getTargetPosition().isSC()
                                                && armistice.armAndElevatorAtTarget().getAsBoolean())),
                                armistice.runToPositionNoWait(ArmisticePositions.STOW)
                                        .onlyIf(() -> armistice.getTargetPosition() != scorePos.get().toPipe())
                                        .andThen(armistice.waitUntilThingsInTolerance(10, 0.3))
                                        .andThen(Commands.runOnce(() -> armistice.setSafety(false)),
                                                Commands.waitUntil(drive.readyForArm())
                                                        .andThen(armistice.runToPositionNoWait(scorePos.get().toPipe()))
                                                        .andThen(
                                                                Commands.defer(
                                                                                () -> drive.waitForDrivetrainDistance(
                                                                                        switch (scorePos.get().getUnPipe()) {
                                                                                            case Cora_L4 -> 0.5;
                                                                                            case Cora_L3 -> 0.3;
                                                                                            default -> 0.2;
                                                                                        }),
                                                                                Set.of())
                                                                        .andThen(armistice.waitUntilThingsInTolerance(3,
                                                                                0.3))
                                                                        .andThen(
                                                                                armistice.runToPositionNoWait(
                                                                                        scorePos.get().getSCPose()
                                                                                                .toPipe(),
                                                                                        drive.closestReefName(),
                                                                                        drive.getReefTargetIsRight()))
                                                                        .andThen(Commands.defer(
                                                                                () -> armistice
                                                                                        .waitUntilThingsInTolerance(1,
                                                                                                scorePos.get() == ArmisticePositions.Cora_L4
                                                                                                        ? 0.3
                                                                                                        : 0.1),
                                                                                Set.of())))))
                        .andThen(Commands.defer(
                                () -> armistice.waitUntilThingsInTolerance(1,
                                        scorePos.get() == ArmisticePositions.Cora_L4 ? 0.6 : 0.3),
                                Set.of()))
                        .andThen(drive.runVelocityAngle(() -> 0, () -> -3, drive::getRotation)
                                .alongWith(coral.runMotorCommand(-.3))
                                .withTimeout(0.4)
                                .andThen(coral.runMotorCommand(0))
                                .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW)))
                        .finallyDo(() -> armistice.waitUntilThingsInTolerance(3, Units.degreesToRadians(5))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(!(isMagicScoreRunning = false)))
                                        .onlyIf(() -> !isMagicScoreRunning))
                                .schedule()));
    }

    public static Command magicScoreL2NoBackup(Drive drive, Armistice armistice, WhipStick coral,
                                           Supplier<Pose2d> reefPose, Supplier<ArmisticePositions> scorePos) {
        return Commands.runOnce(() -> isMagicScoreRunning = true)
                .andThen(drive.translateToPositionWithPID(reefPose.get())
                        .raceWith(Commands.defer(
                                                () -> drive.waitForDrivetrainDistance(0.006),
                                                Set.of())
                                        .alongWith(Commands.waitUntil(() -> armistice.getTargetPosition().isSC()
                                                && armistice.armAndElevatorAtTarget().getAsBoolean())),
                                armistice.runToPositionNoWait(ArmisticePositions.Cora_L2_PIPE)
                                        .andThen(armistice.waitUntilThingsInTolerance(10, 0.3))
                                        .andThen(Commands.runOnce(() -> armistice.setSafety(false)),
                                                Commands.waitUntil(drive.readyForArm(scorePos.get()))
                                                        .andThen(armistice.runToPositionNoWait(scorePos.get().toPipe()))
                                                        .andThen(
                                                                Commands.defer(
                                                                                () -> drive.waitForDrivetrainDistance(
                                                                                        switch (scorePos.get().getUnPipe()) {
                                                                                            case Cora_L4 -> 0.5;
                                                                                            case Cora_L3 -> 0.3;
                                                                                            default -> 0.2;
                                                                                        }),
                                                                                Set.of())
                                                                        .andThen(armistice.waitUntilThingsInTolerance(3,
                                                                                0.3))
                                                                        .andThen(
                                                                                armistice.runToPositionNoWait(
                                                                                        scorePos.get().getSCPose()
                                                                                                .toPipe(),
                                                                                        drive.closestReefName(),
                                                                                        drive.getReefTargetIsRight()))
                                                                        .andThen(Commands.defer(
                                                                                () -> armistice
                                                                                        .waitUntilThingsInTolerance(1,
                                                                                                scorePos.get() == ArmisticePositions.Cora_L4
                                                                                                        ? 0.3
                                                                                                        : 0.1),
                                                                                Set.of())))))
                        .andThen(Commands.defer(
                                () -> armistice.waitUntilThingsInTolerance(1,
                                        scorePos.get() == ArmisticePositions.Cora_L4 ? 0.6 : 0.3),
                                Set.of()))
                        .andThen(drive.runVelocityAngle(() -> 0, () -> -3, drive::getRotation)
                                .alongWith(coral.runMotorCommand(-.3))
                                .withTimeout(0.25)
                                .andThen(coral.runMotorCommand(0))
                                .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW)))
                        .finallyDo(() -> armistice.waitUntilThingsInTolerance(3, Units.degreesToRadians(5))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(!(isMagicScoreRunning = false)))
                                        .onlyIf(() -> !isMagicScoreRunning))
                                .schedule()));
    }

    public static Command magicScore(Drive drive, Armistice armistice, WhipStick coral,
                                          Supplier<Pose2d> reefPose, Supplier<ArmisticePositions> scorePos, BooleanSupplier superCycle) {
        return Commands.runOnce(() -> isMagicScoreRunning = true)
                .andThen(drive.translateToPositionWithPID(reefPose.get())
                        .raceWith(Commands.defer(
                                                () -> drive.waitForDrivetrainDistance(0.006),
                                                Set.of())
                                        .alongWith(Commands.waitUntil(() -> armistice.getTargetPosition().isSC()
                                                && armistice.armAndElevatorAtTarget().getAsBoolean())),
                                armistice.runToPositionNoWait(ArmisticePositions.STOW)
                                        .onlyIf(() -> armistice.getTargetPosition() != scorePos.get().toPipe())
                                        .andThen(armistice.waitUntilThingsInTolerance(10, 0.3))
                                        .andThen(Commands.runOnce(() -> armistice.setSafety(false)),
                                                Commands.waitUntil(drive.readyForArm())
                                                        .andThen(armistice.runToPositionNoWait(scorePos.get().toPipe()))
                                                        .andThen(
                                                                Commands.defer(
                                                                                () -> drive.waitForDrivetrainDistance(
                                                                                        switch (scorePos.get().getUnPipe()) {
                                                                                            case Cora_L4 -> 0.5;
                                                                                            case Cora_L3 -> 0.3;
                                                                                            default -> 0.2;
                                                                                        }),
                                                                                Set.of())
                                                                        .andThen(armistice.waitUntilThingsInTolerance(3,
                                                                                0.3))
                                                                        .andThen(
                                                                                armistice.runToPositionNoWait(
                                                                                        scorePos.get().getSCPose()
                                                                                                .toPipe(),
                                                                                        drive.closestReefName(),
                                                                                        drive.getReefTargetIsRight()))
                                                                        .andThen(Commands.defer(
                                                                                () -> armistice
                                                                                        .waitUntilThingsInTolerance(1,
                                                                                                scorePos.get() == ArmisticePositions.Cora_L4
                                                                                                        ? 0.3
                                                                                                        : 0.1),
                                                                                Set.of())))))
                        .andThen(Commands.defer(
                                () -> armistice.waitUntilThingsInTolerance(1,
                                        scorePos.get() == ArmisticePositions.Cora_L4 ? 0.6 : 0.3),
                                Set.of()))
                        .andThen(drive.runVelocityAngle(() -> 0, () -> -1, drive::getRotation)
                                .alongWith(coral.runMotorCommand(-.3))
                                .withTimeout(superCycle.getAsBoolean() ? 0.05 : .3)
                                .andThen(coral.runMotorCommand(0))
                                .alongWith(Commands.waitSeconds(0.5)
                                        .onlyIf(() -> scorePos.get().getUnPipe() == ArmisticePositions.Cora_L2)
                                        .andThen(armistice.runToPositionNoWait(ArmisticePositions.STOW))
                                        .either(armistice.runToPositionNoWait(armistice.getAutoAlgaePosition(),
                                                        drive.closestReefName(), drive.getReefTargetIsRight()),
                                                superCycle.bsnot()))
                                .andThen(armistice.waitUntilThingsInTolerance(3, Units.degreesToRadians(5))))
                        .finallyDo(() -> armistice.setSafety(!(isMagicScoreRunning = false))));
    }

    public static Command magicScoreNoStow(Drive drive, Armistice armistice, WhipStick coral,
                                                          Supplier<Pose2d> reefPose, Supplier<ArmisticePositions> scorePos, BooleanSupplier superCycle) {
        return Commands.runOnce(() -> isMagicScoreRunning = true)
                .andThen(drive.translateToPositionWithPID(reefPose.get())
                        .raceWith(Commands.defer(
                                                () -> drive.waitForDrivetrainDistance(0.006),
                                                Set.of())
                                        .alongWith(Commands.waitUntil(() -> armistice.getTargetPosition().isSC()
                                                && armistice.armAndElevatorAtTarget().getAsBoolean())),
                                Commands.runOnce(() -> armistice.setSafety(false))
                                        .andThen(Commands.waitUntil(drive.readyForArm())
                                                .andThen(armistice.runToPositionNoWait(scorePos.get().toPipe()))
                                                .andThen(
                                                        Commands.defer(
                                                                        () -> drive.waitForDrivetrainDistance(
                                                                                switch (scorePos.get().getUnPipe()) {
                                                                                    case Cora_L4 -> 0.5;
                                                                                    case Cora_L3 -> 0.3;
                                                                                    default -> 0.2;
                                                                                }),
                                                                        Set.of())
                                                                .andThen(armistice.waitUntilThingsInTolerance(3,
                                                                        0.3))
                                                                .andThen(
                                                                        armistice.runToPositionNoWait(
                                                                                scorePos.get().getSCPose()
                                                                                        .toPipe(),
                                                                                drive.closestReefName(),
                                                                                drive.getReefTargetIsRight()))
                                                                .andThen(Commands.defer(
                                                                        () -> armistice
                                                                                .waitUntilThingsInTolerance(1,
                                                                                        // scorePos.get() ==
                                                                                        // ArmisticePositions.Cora_L4
                                                                                        // ? 15
                                                                                        // : 1,
                                                                                        scorePos.get() == ArmisticePositions.Cora_L4
                                                                                                ? 0.3
                                                                                                : 0.1),
                                                                        Set.of())))))
                        .andThen(Commands.defer(
                                () -> armistice.waitUntilThingsInTolerance(1,
                                        scorePos.get() == ArmisticePositions.Cora_L4 ? 0.6 : 0.3),
                                Set.of()))
                        .andThen(drive.runVelocityAngle(() -> 0, () -> -1, drive::getRotation)
                                .alongWith(coral.runMotorCommand(-.3))
                                .withTimeout(superCycle.getAsBoolean() ? 0.08 : .3)
                                .andThen(coral.runMotorCommand(0))
                                .alongWith(Commands.waitSeconds(0.25)
                                        .onlyIf(() -> scorePos.get().getUnPipe() == ArmisticePositions.Cora_L2)
                                        .andThen(armistice.runToPositionNoWait(ArmisticePositions.STOW))
                                        .either(armistice.runToPositionNoWait(armistice.getAutoAlgaePosition(),
                                                        drive.closestReefName(), drive.getReefTargetIsRight()),
                                                superCycle.bsnot()))
                                .andThen(armistice.waitUntilThingsInTolerance(3, Units.degreesToRadians(5))))
                        .finallyDo(() -> armistice.setSafety(!(isMagicScoreRunning = false))));
    }

    public static Command magicAlgae(Drive drive, Armistice armistice, WhipStick algae,
                                               Supplier<Pose2d> reefPosition, Supplier<ArmisticePositions> acquirePosition, BooleanSupplier superCycle) {
        return Commands.runOnce(() -> {
            isMagicScoreRunning = true;
            algae.setHasGamepiece(false);
        }).alongWith(algae.runMotorCommand(0)).andThen(armistice
                        .runToPositionNoWait(ArmisticePositions.STOW)
                        .andThen(armistice.waitUntilThingsInTolerance(10, 0.3)).onlyIf(superCycle.bsnot())
                        .andThen(Commands.runOnce(() -> armistice.setSafety(false)))
                        .andThen(armistice.runToPositionNoWait(acquirePosition.get()))
                        .onlyIf(() -> armistice.getTargetPosition() != acquirePosition.get())
                        .andThen(armistice.waitUntilThingsInTolerance(3, 0.3).alongWith(drive.waitForDrivetrainDistance(0.5)))
                        .raceWith(drive
                                .translateToPositionWithPID(
                                        reefPosition.get()
                                                .transformBy(new Transform2d(new Translation2d(-0.35, 0)
                                                        .rotateBy(Constants.SCORING_SIDE_FROM_FRONT_ROT), Rotation2d.kZero)))
                                .onlyIf(superCycle.bsnot()))
                        .andThen(
                                drive.translateToPositionWithPID(reefPosition.get()
                                                .transformBy(new Transform2d(new Translation2d(Units.inchesToMeters(0), 0)
                                                        .rotateBy(Constants.SCORING_SIDE_FROM_FRONT_ROT), Rotation2d.kZero)))
                                        .alongWith(algae.runMotorCommandAlgae(0.95))
                                        .until(algae.hasGamePieceSupplier()).withTimeout(3))
                        .andThen(drive.runVelocityAngle(() -> 0, () -> -2, drive::getRotation).withTimeout(0.5)
                                .andThen(armistice.runToPositionNoWait(ArmisticePositions.STOW).withTimeout(.3)
                                        .alongWith(drive.runOnce(drive::stop)))))
                .finallyDo(() -> armistice.setSafety(!(isMagicScoreRunning = false)));
    }
}