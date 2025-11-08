package frc.robot.commands;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Armistice;
import frc.robot.Constants;
import frc.robot.Armistice.ArmisticePositions;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.stick.WhipStick;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod(DriveCommands.class)
public class MagicSequencing {

        public static boolean isMagicScoreRunning = false;
        private static double DRIVE_TOL = 0.01;

        public static Command magicScore(Drive drive,
                        Armistice armistice,
                        WhipStick coral,
                        Supplier<Pose2d> reefPose,
                        Supplier<ArmisticePositions> scorePos,
                        BooleanSupplier superCycle) {
                return Commands.runOnce(() -> isMagicScoreRunning = true)
                                .andThen(drive.translateToPositionWithPID(reefPose.get())
                                                .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW))
                                                .raceWith(Commands.defer(
                                                                () -> drive.waitForDrivetrainDistance(DRIVE_TOL),
                                                                Set.of())))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(false))
                                                .alongWith(armistice.runToPositionNoWait(scorePos.get().toPipe()))
                                                .alongWith(Commands.runOnce(() -> SmartDashboard.putString("MagicScore",
                                                                "ScoringAt " + scorePos.get())))
                                                .andThen(armistice.waitUntilThingsInTolerance(2,
                                                                Units.degreesToRadians(2)))
                                                .andThen(armistice.runToPositionNoWait(scorePos.get().getSCPose()))
                                                .andThen(armistice.waitUntilThingsInTolerance(2,
                                                                Units.degreesToRadians(2))))
                                .andThen(coral.runMotorCommand(-0.3)
                                                .alongWith(drive.runVelocityAngle(() -> 0, () -> -2,
                                                                drive::getRotation))
                                                .withTimeout(0.3)
                                                .andThen(coral.stopMotorCommand()
                                                                .alongWith(Commands.runOnce(() -> drive.stop())))
                                                .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW)))
                                .andThen(drive.runVelocityAngle(() -> 0, () -> -1, drive::getRotation)
                                                .alongWith(Commands.runOnce(() -> armistice.setSafety(true)))
                                                .alongWith(Commands.runOnce(() -> isMagicScoreRunning = false))
                                                .withTimeout(0.3));
        }

        public static Command magicScoreNoStow(Drive drive,
                        Armistice armistice,
                        WhipStick coral,
                        Supplier<Pose2d> reefPose,
                        Supplier<ArmisticePositions> scorePos,
                        BooleanSupplier superCycle) {
                return Commands.runOnce(() -> isMagicScoreRunning = true)
                                .andThen(drive.translateToPositionWithPID(reefPose.get()))
                                .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW))
                                .raceWith(Commands.defer(
                                                () -> drive.waitForDrivetrainDistance(DRIVE_TOL),
                                                Set.of()))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(false)))
                                .alongWith(armistice.runToPositionNoWait(scorePos.get().toPipe()))
                                .andThen(armistice.waitUntilThingsInTolerance(3, 0.1))
                                .andThen(armistice.runToPositionNoWait(scorePos.get().getSCPose()))
                                .andThen(armistice.waitUntilThingsInTolerance(3, 0.1))
                                .andThen(coral.runMotorCommand(0.3))
                                .alongWith(drive.runVelocityAngle(() -> 0, () -> -2, drive::getRotation))
                                .withTimeout(0.3)
                                .andThen(coral.stopMotorCommand())
                                .alongWith(Commands.runOnce(() -> drive.stop()))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(true)))
                                .alongWith(Commands.runOnce(() -> isMagicScoreRunning = false));
        }

        public static Command magicScoreNoBackup(Drive drive,
                        Armistice armistice,
                        WhipStick coral,
                        Supplier<Pose2d> reefPose,
                        Supplier<ArmisticePositions> scorePos) {
                return Commands.runOnce(() -> isMagicScoreRunning = true)
                                .andThen(drive.translateToPositionWithPID(reefPose.get()))
                                .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW))
                                .raceWith(Commands.defer(
                                                () -> drive.waitForDrivetrainDistance(DRIVE_TOL),
                                                Set.of()))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(false)))
                                .alongWith(armistice.runToPositionNoWait(scorePos.get().toPipe()))
                                .andThen(armistice.waitUntilThingsInTolerance(3, 0.1))
                                .andThen(armistice.runToPositionNoWait(scorePos.get().getSCPose()))
                                .andThen(armistice.waitUntilThingsInTolerance(3, 0.1))
                                .andThen(coral.runMotorCommand(0.3))
                                .alongWith(drive.runVelocityAngle(() -> 0, () -> -2, drive::getRotation))
                                .withTimeout(0.3)
                                .andThen(coral.stopMotorCommand())
                                .alongWith(Commands.runOnce(() -> drive.stop()))
                                .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(true)))
                                .alongWith(Commands.runOnce(() -> isMagicScoreRunning = false));
        }

        public static Command magicScoreL2NoBackup(Drive drive,
                        Armistice armistice,
                        WhipStick coral,
                        Supplier<Pose2d> reefPose,
                        Supplier<ArmisticePositions> scorePos) {
                return Commands.runOnce(() -> isMagicScoreRunning = true)
                                .andThen(drive.translateToPositionWithPID(reefPose.get()))
                                .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW))
                                .raceWith(Commands.defer(
                                                () -> drive.waitForDrivetrainDistance(DRIVE_TOL),
                                                Set.of()))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(false)))
                                .alongWith(armistice.runToPositionNoWait(scorePos.get().Cora_L2_PIPE))
                                .andThen(armistice.waitUntilThingsInTolerance(3, 0.01))
                                .andThen(armistice.runToPositionNoWait(scorePos.get().Cora_L2_PIPE_SC))
                                .andThen(armistice.waitUntilThingsInTolerance(3, 0.1))
                                .andThen(coral.runMotorCommand(0.3))
                                .alongWith(drive.runVelocityAngle(() -> 0, () -> -2, drive::getRotation))
                                .withTimeout(0.3)
                                .andThen(coral.stopMotorCommand())
                                .alongWith(Commands.runOnce(() -> drive.stop()))
                                .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(true)))
                                .alongWith(Commands.runOnce(() -> isMagicScoreRunning = false));
        }

        public static Command magicScoreSafeNoBackup(Drive drive,
                        Armistice armistice,
                        WhipStick coral,
                        Supplier<Pose2d> reefPose,
                        Supplier<ArmisticePositions> scorePos) {
                return Commands.runOnce(() -> isMagicScoreRunning = true)
                                .andThen(drive.translateToPositionWithPID(reefPose.get()))
                                .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW))
                                .raceWith(Commands.defer(
                                                () -> drive.waitForDrivetrainDistance(DRIVE_TOL),
                                                Set.of()))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(true)))
                                .alongWith(armistice.runToPositionNoWait(scorePos.get().toPipe()))
                                .andThen(armistice.waitUntilThingsInTolerance(3, 0.1))
                                .andThen(armistice.runToPositionNoWait(scorePos.get().getSCPose()))
                                .andThen(armistice.waitUntilThingsInTolerance(3, 0.1))
                                .andThen(coral.runMotorCommand(0.3))
                                .alongWith(drive.runVelocityAngle(() -> 0, () -> -2, drive::getRotation))
                                .withTimeout(0.3)
                                .andThen(coral.stopMotorCommand())
                                .alongWith(Commands.runOnce(() -> drive.stop()))
                                .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(true)))
                                .alongWith(Commands.runOnce(() -> isMagicScoreRunning = false));
        }

        public static Command magicAlgae(Drive drive,
                        Armistice armistice,
                        WhipStick algae,
                        Supplier<Pose2d> reefPosition,
                        Supplier<ArmisticePositions> acquirePosition,
                        BooleanSupplier superCycle) {
                return Commands.runOnce(() -> isMagicScoreRunning = true)
                                .andThen(armistice.runToPositionNoWait(ArmisticePositions.STOW))
                                .andThen(armistice.waitUntilThingsInTolerance(2, Units.degreesToRadians(2)))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(false)))
                                .andThen(armistice.runToPositionNoWait(acquirePosition.get()))
                                .andThen(armistice.waitUntilThingsInTolerance(2, Units.degreesToRadians(2)))
                                .andThen(algae.runMotorCommand(0.95))
                                .alongWith(drive
                                                .translateToPositionWithPID(
                                                                reefPosition.get()
                                                                                .transformBy(new Transform2d(
                                                                                                new Translation2d(-0.35,
                                                                                                                0)
                                                                                                                .rotateBy(Constants.SCORING_SIDE_FROM_FRONT_ROT),
                                                                                                Rotation2d.kZero))))
                                .raceWith(drive.waitForDrivetrainDistance(DRIVE_TOL))
                                .andThen(
                                                drive.translateToPositionWithPID(reefPosition.get()
                                                                .transformBy(new Transform2d(new Translation2d(
                                                                                Units.inchesToMeters(0), 0)
                                                                                .rotateBy(Constants.SCORING_SIDE_FROM_FRONT_ROT),
                                                                                Rotation2d.kZero))))
                                .until(algae.hasAlgae())
                                .andThen(drive.runVelocityAngle(() -> 0, () -> -1, drive::getRotation)
                                                .withTimeout(0.667)
                                                .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW))
                                                .andThen(armistice.waitUntilThingsInTolerance(2,
                                                                Units.degreesToRadians(2)))
                                                .andThen(Commands.runOnce(() -> armistice.setSafety(true)))
                                                .alongWith(Commands.runOnce(() -> isMagicScoreRunning = false)));
        }

        public static Command superCycle(Drive drive,
                        Armistice armistice,
                        WhipStick coral,
                        Supplier<ArmisticePositions> acquirePosition,
                        Supplier<Pose2d> reefPosition,
                        Supplier<Pose2d> reefPose,
                        Supplier<ArmisticePositions> scorePos) {
                return Commands.runOnce(() -> isMagicScoreRunning = true)
                                .andThen(drive.translateToPositionWithPID(reefPose.get())
                                                .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW))
                                                .raceWith(Commands.defer(
                                                                () -> drive.waitForDrivetrainDistance(DRIVE_TOL),
                                                                Set.of())))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(false))
                                                .alongWith(armistice.runToPositionNoWait(scorePos.get().toPipe()))
                                                .alongWith(Commands.runOnce(() -> SmartDashboard.putString("MagicScore",
                                                                "Scoring at: " + scorePos.get())))
                                                .andThen(armistice.waitUntilThingsInTolerance(3, 0.1))
                                                .andThen(armistice.runToPositionNoWait(scorePos.get().getSCPose()))
                                                .andThen(armistice.waitUntilThingsInTolerance(3, 0.1)))
                                .andThen(coral.runMotorCommand(0.3)
                                                .alongWith(drive.runVelocityAngle(() -> 0, () -> -2, drive::getRotation)
                                                                .withTimeout(0.3))
                                                .andThen(coral.stopMotorCommand())
                                                .alongWith(Commands.runOnce(() -> drive.stop()))
                                                .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW))
                                                .andThen(armistice.waitUntilThingsInTolerance(3, 0.1)
                                                                .withTimeout(0.3)))
                                .andThen(armistice.runToPositionNoWait(acquirePosition.get())
                                                .andThen(armistice.waitUntilThingsInTolerance(3, 0.1)))
                                .andThen(coral.runMotorCommand(0.95)
                                                .alongWith(drive
                                                                .translateToPositionWithPID(
                                                                                reefPosition.get()
                                                                                                .transformBy(new Transform2d(
                                                                                                                new Translation2d(
                                                                                                                                -0.35,
                                                                                                                                0)
                                                                                                                                .rotateBy(Constants.SCORING_SIDE_FROM_FRONT_ROT),
                                                                                                                Rotation2d.kZero))))
                                                .raceWith(drive.waitForDrivetrainDistance(DRIVE_TOL))
                                                .andThen(
                                                                drive.translateToPositionWithPID(reefPosition.get()
                                                                                .transformBy(new Transform2d(
                                                                                                new Translation2d(Units
                                                                                                                .inchesToMeters(0),
                                                                                                                0)
                                                                                                                .rotateBy(Constants.SCORING_SIDE_FROM_FRONT_ROT),
                                                                                                Rotation2d.kZero))))
                                                .until(coral.hasAlgae()))
                                .andThen(coral.stopMotorCommand()
                                                .alongWith(drive.runVelocityAngle(() -> 0, () -> -3, drive::getRotation)
                                                                .withTimeout(0.667))
                                                .alongWith(armistice.runToPositionNoWait(ArmisticePositions.STOW))
                                                .andThen(armistice.waitUntilThingsInTolerance(3, 0.1)))
                                .andThen(Commands.runOnce(() -> armistice.setSafety(true))
                                                .alongWith(Commands.runOnce(() -> isMagicScoreRunning = false)));
        }
}