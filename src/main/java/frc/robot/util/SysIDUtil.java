package frc.robot.util;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;

public class SysIDUtil {
    public static void logSysIdState(State state) {
        SignalLogger.writeString("test-mode", state.toString());
    }
}
