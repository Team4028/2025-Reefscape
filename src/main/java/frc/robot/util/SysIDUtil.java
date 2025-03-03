package frc.robot.util;

import java.util.Map;
import java.util.function.DoubleConsumer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class SysIDUtil {
    private static final boolean USE_AK_LOGGER = false;

    public static void logSysIdState(State state) {
        if (USE_AK_LOGGER)
            Logger.recordOutput("test-mode", state.toString());
        else
            SignalLogger.writeString("test-mode", state.toString());
    }

    public static SysIdRoutine.Config defaultConfig() {
        return new SysIdRoutine.Config(null, null, null, SysIDUtil::logSysIdState);
    }

    /**
     * Boolean value: dynamic (true) or quasistatic (false)
     */
    public static Map<Boolean, Map<Direction, Command>> generateTests(SysIdRoutine.Config config, DoubleConsumer drive,
            Subsystem subsystem) {
        var mech = new SysIdRoutine.Mechanism(v -> drive.accept(v.magnitude()), null, subsystem);
        var sys = new SysIdRoutine(config, mech);
        return Map.of(
            true, Map.of(Direction.kForward, sys.dynamic(Direction.kForward), Direction.kReverse,
            sys.dynamic(Direction.kReverse)),
            false, Map.of(Direction.kForward, sys.quasistatic(Direction.kForward), Direction.kReverse,
            sys.quasistatic(Direction.kReverse)));
    }
}
