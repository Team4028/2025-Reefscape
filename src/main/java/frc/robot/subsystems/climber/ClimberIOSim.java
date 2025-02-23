package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.util.RobotSim;

/*
 * 
 * THIS IS STILL A WORK IN PROGRESS
 * 
 */
public class ClimberIOSim implements ClimberIO {
    private static FlywheelSim climber;

    public ClimberIOSim() {
        climber = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(ClimberConstants.Sim.simGearbox,
                        ClimberConstants.Sim.CLIMBER_KgMSquared, ClimberConstants.GEAR_RATIO),
                ClimberConstants.Sim.simGearbox);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        climber.update(0.02);
    }

    @Override
    public void setVoltage(double volts) {

    }

    @Override
    public void setVbus(double vBus) {

    }

    @Override
    public void setPid(double position) {

    }
}
