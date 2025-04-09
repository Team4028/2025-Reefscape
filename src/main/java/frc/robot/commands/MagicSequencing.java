package frc.robot.commands;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.function.BooleanConsumer;
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

@ExtensionMethod({ MiscUtils.class, DriveCommands.class, Commands.class })
@UtilityClass
public class MagicSequencing {
    public static final Command magicScoreScore(Drive drive, Armistice armistice, WhipStick coral,
            Supplier<Pose2d> reefPose, Supplier<ArmisticePositions> scorePos, BooleanSupplier superCycle) {
        return drive.translateToPositionWithPID(reefPose.get())
                .raceWith(Commands.defer(
                        () -> drive.waitForDrivetrainDistance(0.006),
                        Set.of())
                        .alongWith(Commands.waitUntil(() -> armistice.getTargetPosition().isSC()
                                && armistice.armAndElevatorAtTarget().getAsBoolean())),
                        armistice.runToPositionNoWait(ArmisticePositions.STOW)
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
                                                                .andThen(armistice.waitUntilThingsInTolerance(3, 0.3))
                                                                .andThen(
                                                                        armistice.runToPositionNoWait(
                                                                                scorePos.get().getSCPose().toPipe(),
                                                                                drive.closestReefName(),
                                                                                drive.getReefTargetIsRight()))
                                                                .andThen(Commands.defer(
                                                                        () -> armistice.waitUntilThingsInTolerance(1,
                                                                                scorePos.get() == ArmisticePositions.Cora_L4
                                                                                        ? 0.3
                                                                                        : 0.1),
                                                                        Set.of())))))
                .andThen(Commands.defer(
                        () -> armistice.waitUntilThingsInTolerance(1,
                                scorePos.get() == ArmisticePositions.Cora_L4 ? 0.6 : 0.3),
                        Set.of()))
                .andThen(drive.runVelocityAngle(() -> 0, () -> -1, () -> drive.getRotation())
                        .alongWith(coral.runMotorCommand(-.3))
                        .withTimeout(0.3)
                        .andThen(coral.runMotorCommand(0))
                        .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW)
                                .either(armistice.runToPositionNoWait(armistice.getAutoAlgaePosition(),
                                        drive.closestReefName(), drive.getReefTargetIsRight()), superCycle.not()))
                        .andThen(armistice.waitUntilThingsInTolerance(3, Units.degreesToRadians(5))))
                .finallyDo(() -> armistice.setSafety(true));
    }

    public static final Command magicGetAlgaeOnlyPID(Drive drive, Armistice armistice, WhipStick algae,
            Supplier<Pose2d> reefPosition, Supplier<ArmisticePositions> acquirePosition, BooleanSupplier superCycle) {
        return armistice.runToPositionNoWait(ArmisticePositions.STOW)
                .andThen(armistice.waitUntilThingsInTolerance(10, 0.3)).onlyIf(superCycle.not())
                .andThen(Commands.runOnce(() -> armistice.setSafety(false)))
                .andThen(armistice.runToPositionNoWait(acquirePosition.get()))
                .andThen(armistice.waitUntilThingsInTolerance(3, 0.3).alongWith(drive.waitForDrivetrainDistance(0.5)))
                .raceWith(drive
                        .translateToPositionWithPID(
                                reefPosition.get()
                                        .transformBy(new Transform2d(new Translation2d(-0.35, 0)
                                                .rotateBy(Constants.SCORING_SIDE_FROM_FRONT_ROT), Rotation2d.kZero))))
                .andThen(
                        drive.translateToPositionWithPID(reefPosition.get()
                                .transformBy(new Transform2d(new Translation2d(Units.inchesToMeters(0), 0)
                                        .rotateBy(Constants.SCORING_SIDE_FROM_FRONT_ROT), Rotation2d.kZero)))
                                .alongWith(algae.runMotorCommandAlgae(0.95))
                                .until(algae.hasGamePieceSupplier()).withTimeout(3))
                .andThen(drive.runVelocityAngle(() -> 0, () -> -2, () -> drive.getRotation()).withTimeout(0.3)
                        .andThen(armistice.runToPositionNoWait(ArmisticePositions.STOW)
                                .alongWith(drive.runOnce(drive::stop)))
                        .andThen(armistice.waitUntilThingsInTolerance(3, Units.degreesToRadians(5))))
                .finallyDo(() -> {
                    armistice.setSafety(true);
                });

    }
}