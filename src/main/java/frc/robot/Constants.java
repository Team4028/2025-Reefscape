// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    public static final double ALGAE_RADIUS_M = 0.206375;
    public static final double ALGAE_WEIGHT_KG = 0.680389;

    public static final int THE_BEST_NUMBER = 4028;

    public static final Mode simMode = Mode.REPLAY;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static enum Mode {
        REAL, SIM, REPLAY
    }
}
