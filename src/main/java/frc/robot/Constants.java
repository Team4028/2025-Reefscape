// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    public static final double ALGAE_RADIUS_M = 0.206375;
    public static final double ALGAE_WEIGHT_KG = 0.680389;

    public static final int THE_BEST_NUMBER = 4028;

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final double TAG_TO_BRANCH_OFFSET_M = 0.185;
    public static final double ARM_READY_AUTO_SCORE_RADIUS = 1.5;
    public static final double SCORING_SIDE_RADIUS_ROBOT_IN = 18.25;
    public static final double CORAL_SCORE_OFFSET_FROM_CENTERLINE_IN = 0.75;
    public static final double CORAL_STATION_LEFT_ROTATION_DEG = 135;
    public static final double CORAL_STATION_RIGHT_ROTATION_DEG = 225;

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
