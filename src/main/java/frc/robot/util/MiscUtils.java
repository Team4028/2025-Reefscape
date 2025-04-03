package frc.robot.util;

import java.util.function.BooleanSupplier;

import lombok.experimental.UtilityClass;

@UtilityClass
public class MiscUtils {
    public static final <T> T orIfNull(T value, T _default) {
        return value == null ? _default : value;
    }

    public static final BooleanSupplier or(BooleanSupplier a, BooleanSupplier b) {
        return () -> a.getAsBoolean() || b.getAsBoolean();
    }

    public static final BooleanSupplier and(BooleanSupplier a, BooleanSupplier b) {
        return () -> a.getAsBoolean() && b.getAsBoolean();
    }

    public static final BooleanSupplier not(BooleanSupplier b) {
        return () -> !b.getAsBoolean();
    }

    public static final boolean[] convertToPrimitive(Boolean[] bArr) {
        boolean[] barray = new boolean[bArr.length];
        for (var i = 0; i < bArr.length; i++) {
            barray[i] = bArr[i];
        }

        return barray;
    }
}