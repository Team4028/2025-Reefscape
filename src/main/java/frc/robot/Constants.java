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

    public static final Mode simMode = Mode.REPLAY;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final double TAG_TO_BRANCH_OFFSET_M = 0.185;
    public static final double ARM_READY_AUTO_SCORE_RADIUS = 1.5;
    public static final double SCORING_SIDE_RADIUS_ROBOT_IN = 18;
    public static final double CORAL_SCORE_OFFSET_FROM_CENTERLINE_IN = 8.25;

    public static final Map<Integer, String> reefTagNames = Map.of(
        6, "8oC",
        7, "6oC",
        8, "4oC",
        9, "2oC",
        10, "12oC",
        11, "10oC"
    );

    static {
        reefTagNames.putAll(Map.of(
            17, "4oC",
        18, "6oC",
        19, "8oC",
        20, "10oC",
        21, "12oC",
        22, "2oC"
        ));
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kEmergencyControllerPort = 2;
    }

    public static enum Mode {
        REAL, SIM, REPLAY
    }
}
