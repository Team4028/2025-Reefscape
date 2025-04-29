// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.experimental.UtilityClass;

import java.util.Arrays;

@UtilityClass
public class MathUtils {
    public static <T extends Number> T clamp(T value, T min, T max) {
        if (value.doubleValue() < min.doubleValue())
            return min;
        else if (value.doubleValue() > max.doubleValue())
            return max;
        else
            return value;
    }

    public static double average(double... numbers) {
        double sum = 0;
        for (double n : numbers) sum += n;
        return sum / numbers.length;
    }
    
    public static double multOfPI(double mul) {
        return Math.PI * mul;
    }

    public static double cyclic(double value, double period) {
        var tempVal = value;
        while (tempVal < 0)
            tempVal += period;
        return tempVal % period;
    }

    public static double cyclicRange(double value, double low, double high, double period) {
        var tempValue = value;
        while (tempValue < low || tempValue > high) {
            if (tempValue > high)
                tempValue -= period;
            else tempValue += period;
        }

        return tempValue;
    }

    public static double get2dVelocity(ChassisSpeeds chassisSpeeds) {
        return Math.sqrt(chassisSpeeds.vxMetersPerSecond * chassisSpeeds.vxMetersPerSecond
                + chassisSpeeds.vyMetersPerSecond * chassisSpeeds.vyMetersPerSecond);
    }

    public static <T extends Number> boolean inRange(T value, T minInc, T maxInc) {
        return value.doubleValue() >= minInc.doubleValue() && value.doubleValue() <= maxInc.doubleValue();
    }

    public static <T extends Number> boolean inRangeWithTolerance(T value, T minInc, T maxInc, T tolerance) {
        return value.doubleValue() + tolerance.doubleValue() >= minInc.doubleValue()
                && value.doubleValue() - tolerance.doubleValue() <= maxInc.doubleValue();
    }

    public static double roundToPlace(double x, int place) {
        var pow10 = Math.pow(10, place);
        return Math.round(x * pow10) / pow10;
    }

    public static int boolToInt(boolean b) {
        return b ? 1 : 0;
    }

    public static double[] rotateVector(double[] vec, double thetaRad) {
        return new double[] {
                vec[0] * Math.cos(thetaRad) - vec[1] * Math.sin(thetaRad),
                vec[0] * Math.sin(thetaRad) + vec[1] * Math.cos(thetaRad),
        };
    }
}
