// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class MathUtil {
    public static final <T extends Number> T clamp(T value, T min, T max) {
        if (value.doubleValue() < min.doubleValue())
            return min;
        else if (value.doubleValue() > max.doubleValue())
            return max;
        else
            return value;
    }

    public static final int boolToInt(boolean b) {
        return b ? 1 : 0;
    }

    public static final double[] rotateVector(double[] vec, double thetaRad) {
        return new double[] {
            vec[0] * Math.cos(thetaRad) - vec[1] * Math.sin(thetaRad),
            vec[0] * Math.sin(thetaRad) + vec[1] * Math.cos(thetaRad),
        };
    }
}
