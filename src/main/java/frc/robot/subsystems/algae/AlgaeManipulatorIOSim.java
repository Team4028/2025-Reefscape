package frc.robot.subsystems.algae;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.coral.CoralManipulatorConstants;
import frc.robot.subsystems.coral.CoralManipulatorIO.CoralManipulatorIOInputs;
import frc.robot.util.GetMotorData.MotorData;

public class AlgaeManipulatorIOSim implements AlgaeManipulatorIO {
    private final FlywheelSim manip;
    private double targetVolts = 0;

    public AlgaeManipulatorIOSim() {
        manip = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(AlgaeManipulatorConstants.Sim.simGearbox,
                        AlgaeManipulatorConstants.Sim.MANIP_MOI_KgMSquared, AlgaeManipulatorConstants.GEARING),
                AlgaeManipulatorConstants.Sim.simGearbox);
    }

    @Override
    public void updateInputs(AlgaeManipulatorIOInputs inputs) {
        manip.setInput(targetVolts);
        manip.update(0.02);
        inputs.appliedVolts = targetVolts;
        inputs.currentAmps = manip.getCurrentDrawAmps();
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(manip.getCurrentDrawAmps()));
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
