package frc.robot.subsystems.coral;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class CoralManipulatorIOSim implements CoralManipulatorIO {
    private final FlywheelSim manip;
    private double lastVolts = 0.0;

    public CoralManipulatorIOSim() {
        manip = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(CoralManipulatorConstants.Sim.simGearbox,
                        CoralManipulatorConstants.Sim.MANIP_MOI_KgMSquared, CoralManipulatorConstants.GEARING),
                CoralManipulatorConstants.Sim.simGearbox);
    }

    @Override
    public void updateInputs(CoralManipulatorIOInputs inputs) {
        manip.update(0.02);
        inputs.appliedVolts = lastVolts;
        inputs.currentAmps = manip.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts) {
        lastVolts = volts;
        manip.setInput(volts);
    }

    @Override
    public void setVbus(double vBus) {
        setVoltage(vBus * RobotController.getBatteryVoltage());
    }
}
