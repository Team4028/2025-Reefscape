package frc.robot.util;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.coral.CoralManipulatorIO;
import frc.robot.subsystems.coral.CoralManipulatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;

public class RobotSim {
    public static final ArmIO armSimSwitch(ArmIO realArm) {
        return Robot.isReal() ? realArm : new ArmIOSim();
    }

    public static final ElevatorIO elevatorSimSwitch(ElevatorIO realElevator) {
        return Robot.isReal() ? realElevator : new ElevatorIOSim();
    }

    public static final CoralManipulatorIO coralManipulatorSimSwitch(CoralManipulatorIO realCoralManipulator) {
        return Robot.isReal() ? realCoralManipulator : new CoralManipulatorIOSim();
    }

    private static final Mechanism2d baseMech = new Mechanism2d(5, 5);
    private static final MechanismRoot2d elevatorRoot = baseMech.getRoot("ElevatorRoot", 2.5, 0);

    @SuppressWarnings("unused")
    private static final MechanismLigament2d elevatorMech = elevatorRoot
            .append(new MechanismLigament2d("Elevator", 5, 0));
    private static final MechanismRoot2d armRoot = baseMech.getRoot("ArmRoot", 2.5, 0);
    private static final MechanismLigament2d armMech = armRoot.append(new MechanismLigament2d("Arm", 2, 0));

    public static final void update(double elevatorHeightMetres, double armAngleRad, double... currentDraw) {
        armRoot.setPosition(2.5, elevatorHeightMetres);
        armMech.setAngle(armAngleRad);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(currentDraw));
    }

    public static final Mechanism2d getMechanism() {
        return baseMech;
    }
}
