package frc.robot.subsystems.coral;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.RobotSim;

public class CoralManipulatorIOSim implements CoralManipulatorIO {
    private final FlywheelSim manip;
    private double targetVolts = 0.0;

    public CoralManipulatorIOSim() {
        manip = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(CoralManipulatorConstants.Sim.simGearbox,
                        CoralManipulatorConstants.Sim.MANIP_MOI_KgMSquared, CoralManipulatorConstants.GEARING),
                CoralManipulatorConstants.Sim.simGearbox);
        RobotSim.registerCurrentInput("Coral Manipulator", manip::getCurrentDrawAmps);
    }

    @Override
    public void updateInputs(CoralManipulatorIOInputs inputs) {
        manip.setInput(targetVolts);
        manip.update(0.02);
        inputs.appliedVolts = targetVolts;
        inputs.currentAmps = manip.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts) {
        targetVolts = volts;
    }

    @Override
    public void setVbus(double vBus) {
        setVoltage(vBus * RobotController.getBatteryVoltage());
    }
}
