package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.util.Units;
import frc.robot.Armistice.SimData;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.cage.Cage;
import frc.robot.subsystems.cage.CageIO;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.stick.WhipStick;
import frc.robot.subsystems.stick.WhipStickIO;
import frc.robot.subsystems.stick.WhipStickIOSim;
import lombok.experimental.UtilityClass;

@UtilityClass
public class RobotSim {
    public static final Map<String, DoubleSupplier> currentInputs = new HashMap<>();

    public static void registerCurrentInput(String key, DoubleSupplier currentSupplier) {
        if (currentInputs.putIfAbsent(key, currentSupplier) != null)
            currentInputs.replace(key, currentSupplier);
    }

    public static Drive simSwitch(GyroIO io, ModuleIO[] moduleIOs) {
        return Constants.currentMode == Mode.REAL
                ? new Drive(io, moduleIOs[0], moduleIOs[1], moduleIOs[2], moduleIOs[3])
                : new Drive(new GyroIO() {
                }, new ModuleIOSim(TunerConstants.FrontLeft), new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft), new ModuleIOSim(TunerConstants.BackRight));
    }

    public static Arm simSwitch(ArmIO realArm) {
        return new Arm(Constants.currentMode == Mode.REAL ? realArm : new ArmIOSim());
    }

    public static Elevator simSwitch(ElevatorIO realElevator) {
        return new Elevator(Constants.currentMode == Mode.REAL ? realElevator : new ElevatorIOSim());
    }

    public static Climber simSwitch(ClimberIO realClimber) {
        return new Climber(Constants.currentMode == Mode.REAL ? realClimber : new ClimberIOSim());
    }


    public static WhipStick simSwitch(WhipStickIO realCoral) {
        return new WhipStick(Constants.currentMode == Mode.REAL ? realCoral : new WhipStickIOSim());
    }

    private static final LoggedMechanism2d baseMech = new LoggedMechanism2d(5, 5);
    private static final LoggedMechanismRoot2d elevatorRoot = baseMech.getRoot("ElevatorRoot", 2.5, 0);

    @SuppressWarnings("unused")
    private static final LoggedMechanismLigament2d elevatorMech = elevatorRoot
            .append(new LoggedMechanismLigament2d("Elevator", Units.inchesToMeters(ElevatorConstants.MAX_HEIGHT_INCHES),
                    90));
    private static final LoggedMechanismRoot2d armRoot = baseMech.getRoot("ArmRoot", 2.5, 0);
    private static final LoggedMechanismLigament2d armMech = armRoot
            .append(new LoggedMechanismLigament2d("Arm", ArmConstants.ARM_LENGTH_METRES, 0));

    public static void update(SimData simData) {
        armRoot.setPosition(2.5, simData.elevatorPositionMeters());
        armMech.setAngle(Units.radiansToDegrees(simData.armPositionRadians()));
    }

    public static LoggedMechanism2d getMechanism() {
        return baseMech;
    }

    public static void logMechanism() {
        Logger.recordOutput("RobotSim/Mechanism", baseMech);
    }
}
