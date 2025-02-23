package frc.robot.subsystems.algae;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SysIDUtil;

public class AlgaeManipulatorConstants {
    public static final double SUPPLY_LIMIT = 40;
    public static final double GEARING = 10;
    public static final class TalonSRX {
        public static final int CAN_ID = 18;
        public static final boolean INVERT = false;

    }
    public static final class TalonFX {
        public static final int CAN_ID = 18;
        public static final boolean INVERT = false;
    }
 public static final class Sim {
        public static final DCMotor simGearbox = DCMotor.getVex775Pro(1);
        public static final double MANIP_MOI_KgMSquared = 1;
    }

    public static final SysIdRoutine.Config sysIDConfig = SysIDUtil.defaultConfig();
}
