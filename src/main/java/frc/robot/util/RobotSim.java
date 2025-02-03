package frc.robot.util;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.coral.CoralManipulatorIO;
import frc.robot.subsystems.coral.CoralManipulatorIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;

public class RobotSim {
    public static final ArmIO armSimSwitch(ArmIO realArm) {
        return Constants.currentMode == Mode.REAL ? realArm : new ArmIOSim();
    }

    public static final ElevatorIO elevatorSimSwitch(ElevatorIO realElevator) {
        return Constants.currentMode == Mode.REAL ? realElevator : new ElevatorIOSim();
    }

    public static final CoralManipulatorIO coralManipulatorSimSwitch(CoralManipulatorIO realCoralManipulator) {
        return Constants.currentMode == Mode.REAL ? realCoralManipulator : new CoralManipulatorIOSim();
    }

    public static final Drive driveSimSwitch(GyroIO io, ModuleIO[] moduleIOs) {
        return Constants.currentMode == Mode.REAL ? new Drive(io, moduleIOs[0], moduleIOs[1], moduleIOs[2], moduleIOs[3])
                : new Drive(new GyroIO() {
                }, new ModuleIOSim(TunerConstants.FrontLeft), new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft), new ModuleIOSim(TunerConstants.BackRight));
    }

    private static LoggedMechanism2d baseMech = new LoggedMechanism2d(5, 5);
    private static LoggedMechanismRoot2d elevatorRoot = baseMech.getRoot("ElevatorRoot", 2.5, 0);

    @SuppressWarnings("unused")
    private static LoggedMechanismLigament2d elevatorMech = elevatorRoot
            .append(new LoggedMechanismLigament2d("Elevator", 5, 90));
    private static LoggedMechanismRoot2d armRoot = baseMech.getRoot("ArmRoot", 2.5, 0);
    private static LoggedMechanismLigament2d armMech = armRoot.append(new LoggedMechanismLigament2d("Arm", 2, 0));

    public static final void update(double elevatorHeightMetres, double armAngleRad) {
        armRoot.setPosition(2.5, elevatorHeightMetres);
        armMech.setAngle(Units.radiansToDegrees(armAngleRad));
    }

    public static final LoggedMechanism2d getMechanism() {
        return baseMech;
    }

    public static final void logMechanism() {
        Logger.recordOutput("RobotSim/Mechanism", baseMech);
    }
}
