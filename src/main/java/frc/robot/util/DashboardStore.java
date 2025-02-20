// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.HashMap;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.util.function.FloatSupplier;

/** Add your docs here. */
public final class DashboardStore {
    private static Map<NetworkTableEntry, Supplier<NetworkTableValue>> values = new HashMap<NetworkTableEntry, Supplier<NetworkTableValue>>();

    @FunctionalInterface
    public static interface ByteArraySupplier {
        byte[] get();
    }

    @FunctionalInterface
    public static interface StringSupplier {
        String get();
    }

    @FunctionalInterface
    public static interface BooleanArraySupplier {
        boolean[] get();
    }

    @FunctionalInterface
    public static interface LongArraySupplier {
        long[] get();
    }

    @FunctionalInterface
    public static interface FloatArraySupplier {
        float[] get();
    }

    @FunctionalInterface
    public static interface DoubleArraySupplier {
        double[] get();
    }

    @FunctionalInterface
    public static interface StringArraySupplier {
        String[] get();
    }

    private static NetworkTableEntry smartDashboardEntry(String key) {
        return NetworkTableInstance.getDefault().getEntry("/SmartDashboard/" + key);
    }

    public static void add(String key, ByteArraySupplier value) {
        values.put(smartDashboardEntry(key), () -> NetworkTableValue.makeRaw(value.get()));
    }

    public static void add(String key, BooleanSupplier value) {
        values.put(smartDashboardEntry(key), () -> NetworkTableValue.makeBoolean(value.getAsBoolean()));
    }

    public static void add(String key, DoubleSupplier value) {
        values.put(smartDashboardEntry(key), () -> NetworkTableValue.makeDouble(value.getAsDouble()));
    }

    public static void add(String key, FloatSupplier value) {
        values.put(smartDashboardEntry(key), () -> NetworkTableValue.makeFloat(value.getAsFloat()));
    }

    public static void add(String key, IntSupplier value) {
        values.put(smartDashboardEntry(key), () -> NetworkTableValue.makeInteger(value.getAsInt()));
    }

    public static void add(String key, StringSupplier value) {
        values.put(smartDashboardEntry(key), () -> NetworkTableValue.makeString(value.get()));
    }

    public static void add(String key, BooleanArraySupplier value) {
        values.put(smartDashboardEntry(key), () -> NetworkTableValue.makeBooleanArray(value.get()));
    }

    public static void add(String key, LongArraySupplier value) {
        values.put(smartDashboardEntry(key), () -> NetworkTableValue.makeIntegerArray(value.get()));
    }

    public static void add(String key, FloatArraySupplier value) {
        values.put(smartDashboardEntry(key), () -> NetworkTableValue.makeFloatArray(value.get()));
    }

    public static void add(String key, DoubleArraySupplier value) {
        values.put(smartDashboardEntry(key), () -> NetworkTableValue.makeDoubleArray(value.get()));
    }

    public static void add(String key, StringArraySupplier value) {
        values.put(smartDashboardEntry(key), () -> NetworkTableValue.makeStringArray(value.get()));
    }

    public static void update() {
        values.forEach((key, value) -> {
            key.setValue(value.get());
        });
    }

}