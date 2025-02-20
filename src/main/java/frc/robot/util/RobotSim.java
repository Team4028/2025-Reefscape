package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.*;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Armistice.SimData;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;

public class RobotSim {

    public static final Map<String, DoubleSupplier> currentInputs = new HashMap<>();

    public static final void registerCurrentInput(String key, DoubleSupplier currentSupplier) {
        if (currentInputs.putIfAbsent(key, currentSupplier) != null)
            currentInputs.replace(key, currentSupplier);
    }

    

    public static final Arm armSimSwitch(ArmIO realArm) {
        return new Arm(Constants.currentMode == Mode.REAL ? realArm : new ArmIOSim());
    }

    public static final Elevator elevatorSimSwitch(ElevatorIO realElevator) {
        return new Elevator(Constants.currentMode == Mode.REAL ? realElevator : new ElevatorIOSim());
    }

    private static LoggedMechanism2d baseMech = new LoggedMechanism2d(5, 5);
    private static LoggedMechanismRoot2d elevatorRoot = baseMech.getRoot("ElevatorRoot", 2.5, 0);

    @SuppressWarnings("unused")
    private static LoggedMechanismLigament2d elevatorMech = elevatorRoot
            .append(new LoggedMechanismLigament2d("Elevator", Units.inchesToMeters(ElevatorConstants.MAX_HEIGHT_INCHES),
                    90));
    private static LoggedMechanismRoot2d armRoot = baseMech.getRoot("ArmRoot", 2.5, 0);
    private static LoggedMechanismLigament2d armMech = armRoot
            .append(new LoggedMechanismLigament2d("Arm", ArmConstants.ARM_LENGTH_METRES, 0));

    public static final void update(SimData simData) {
        armRoot.setPosition(2.5, simData.elevatorPositionMeters());
        armMech.setAngle(Units.radiansToDegrees(simData.armPositionRadians()));
        // RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(currentInputs.values().stream()
        // .mapToDouble(DoubleSupplier::getAsDouble).toArray()));
    }

    public static final LoggedMechanism2d getMechanism() {
        return baseMech;
    }

    public static final void logMechanism() {
        Logger.recordOutput("RobotSim/Mechanism", baseMech);
    }
}
