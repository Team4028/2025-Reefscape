// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    public static final double ALGAE_RADIUS_M = 0.206375;
    public static final double ALGAE_WEIGHT_KG = 0.680389;

    public static final int THE_BEST_NUMBER = 4028;

    public static final Mode simMode = Mode.REPLAY;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static final double TAG_TO_BRANCH_OFFSET_M = 0.185;
    public static final double ARM_READY_AUTO_SCORE_RADIUS = 1.5;

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static enum ReefPositions {
        L2(new Pose2d(5.22, 3.08, Rotation2d.fromDegrees(330))),
        R2(new Pose2d(4.93, 2.92, Rotation2d.fromDegrees(330))),
        L4(new Pose2d(4.02, 2.89, Rotation2d.fromDegrees(30))),
        R4(new Pose2d(3.76, 3.05, Rotation2d.fromDegrees(30))),
        L6(new Pose2d(3.31, 4.21, Rotation2d.fromDegrees(90))),
        R6(new Pose2d(3.30, 3.84, Rotation2d.fromDegrees(90))),
        L8(new Pose2d(4.06, 5.15, Rotation2d.fromDegrees(150))),
        R8(new Pose2d(3.76, 4.97, Rotation2d.fromDegrees(150))),
        L10(new Pose2d(5.25, 4.97, Rotation2d.fromDegrees(210))),
        R10(new Pose2d(4.94, 5.15, Rotation2d.fromDegrees(210))),
        L12(new Pose2d(5.67, 3.83, Rotation2d.fromDegrees(270))),
        R12(new Pose2d(5.67, 4.17, Rotation2d.fromDegrees(270)));

        public Pose2d pose;

        private ReefPositions(Pose2d pose) {
            this.pose = pose;
        }
    }

    public static enum Mode {
        REAL, SIM, REPLAY
    }
}
