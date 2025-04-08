// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    public static final boolean CHAR_MODE = BuildConfig.FIELD_CAL;
    public static final boolean tuningMode = false || CHAR_MODE;
    public static final double ALGAE_RADIUS_M = 0.206375;
    public static final double ALGAE_WEIGHT_KG = 0.680389;
    public static final double CORAL_DIAM_IN = 4.5;
    public static final double BACKUP_DIST_IN = 12;

    public static final boolean USE_ARMISTICE_PID = true && !CHAR_MODE;

    public static final int THE_BEST_NUMBER = 4028;

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final double TAG_TO_BRANCH_OFFSET_M = 0.17;
    public static final double ARM_READY_AUTO_SCORE_RADIUS = 1;
    public static final double SCORING_SIDE_RADIUS_ROBOT_IN = 18.25;
    public static final double CORAL_SCORE_OFFSET_FROM_CENTERLINE_IN = 2.4;
    public static final double ALGAE_SCORE_OFFSET_FROM_CENTERLINE_IN = CORAL_SCORE_OFFSET_FROM_CENTERLINE_IN;
    public static final double CORAL_STATION_LEFT_ROTATION_DEG = 36.86989764584399;
    public static final double CORAL_STATION_RIGHT_ROTATION_DEG = 146.0;
    public static final Rotation2d SCORING_SIDE_FROM_FRONT_ROT = Rotation2d.kCCW_Pi_2;
    public static final Pose2d CENTRAL_POWER_AUTON_START_POSE = new Pose2d(7.2, 4.028, Rotation2d.kCCW_90deg);

    public static final Pose2d AQUIRE_RIGHT_POS = new Pose2d(1.2825183868408203, 0.8288688659667969, Rotation2d.fromRadians(-0.6287963059722557));
    public static final Pose2d AQUIRE_LEFT_POS = new Pose2d(1.2929733991622925, 7.2203779220581055, Rotation2d.fromRadians(3.7805067024924734));

    public static final class Streams {
        public static final String SCORE_CAMERA = "http://photonvision.local:1186/stream.mjpg";
        public static final String CLIMB_CAMERA = "http://photonvision.local:1184/stream.mjpg";
    }

    public static final Map<Integer, String> reefTagNames = new HashMap<>(){{
            put(6, "8oC");
            put(7, "6oC");
            put(8, "4oC");
            put(9, "2oC");
            put(10, "12oC");
            put(11, "10oC");
            put(17, "4oC");
            put(18, "6oC");
            put(19, "8oC");
            put(20, "10oC");
            put(21, "12oC");
            put(22, "2oC");
        }};

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kEmergencyControllerPort = 2;
    }

    public static enum Mode {
        REAL, SIM, REPLAY
    }
}
