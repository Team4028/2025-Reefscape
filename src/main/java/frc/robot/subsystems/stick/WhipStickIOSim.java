package frc.robot.subsystems.stick;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.RobotSim;

public class WhipStickIOSim implements WhipStickIO {
    private final FlywheelSim manip;
    private double targetVolts = 0.0;

    public WhipStickIOSim() {
        manip = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(WhipStickConstants.Sim.simGearbox,
                        WhipStickConstants.Sim.MANIP_MOI_KgMSquared, WhipStickConstants.GEARING),
                WhipStickConstants.Sim.simGearbox);
        RobotSim.registerCurrentInput("Coral Manipulator", manip::getCurrentDrawAmps);
    }

    @Override
    public void updateInputs(WhipStickIOInputs inputs) {
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
