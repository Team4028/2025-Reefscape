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

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

/** Add your docs here. */
public final class DashboardStore {
    private static Map<NetworkTableEntry, Supplier<NetworkTableValue>> values = new HashMap<NetworkTableEntry, Supplier<NetworkTableValue>>();

    @FunctionalInterface
    public static interface ByteArraySupplier {
        byte[] get();
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

    public static void add(String key, IntSupplier value) {
        values.put(smartDashboardEntry(key), () -> NetworkTableValue.makeInteger(value.getAsInt()));
    }

    public static void add(String key, Supplier<String> value) {
        values.put(smartDashboardEntry(key), () -> NetworkTableValue.makeString(value.get()));
    }

    // TODO: add switches and add methods for other array types

    public static void update() {
        values.forEach((key, value) -> {
            key.setValue(value.get());
        });
    }

}