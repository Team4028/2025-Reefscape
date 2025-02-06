package frc.robot.util;

// reference: https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/util/VirtualSubsystem.java

import java.util.ArrayList;
import java.util.List;

public abstract class SudoSubsystem {
    public static List<SudoSubsystem> subsystems = new ArrayList<>();

    public SudoSubsystem() {
        subsystems.add(this);
    }

    public static void periodicAll() {
        subsystems.forEach(SudoSubsystem::periodic);
    }

    public abstract void periodic();
}
