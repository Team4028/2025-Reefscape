package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim arm;
    private double targetVolts = 0.0;

    public ArmIOSim() {
        arm = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(ArmConstants.Sim.simGearbox,
                        ArmConstants.Sim.ARM_MOI_KgMSquared,
                        ArmConstants.GEAR_RATIO),
                ArmConstants.Sim.simGearbox, ArmConstants.GEAR_RATIO, ArmConstants.ARM_LENGTH_METRES, -ArmConstants.PI_2,
                ArmConstants.PI_2,
                true, 0);
            }
            
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        arm.setInput(targetVolts);
        arm.update(0.02);
        inputs.appliedVoltage = targetVolts;
        inputs.armAngleRad = arm.getOutput(0);
        inputs.armEncoderRad = arm.getOutput(0);
        inputs.armEncoderRaw = arm.getOutput(0) / ArmConstants.PI_2;
        inputs.armVelocityRotPerSec = arm.getOutput(1) / ArmConstants.PI_2;
        inputs.armMotorVelocityRotPerSec = inputs.armVelocityRotPerSec / ArmConstants.GEAR_RATIO;
        inputs.currentAmps = arm.getCurrentDrawAmps();
        ArmIO.super.updateInputs(inputs);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(arm.getCurrentDrawAmps()));
    }

    @Override
    public void setVoltage(double volts) {
        targetVolts = volts;
    }

    @Override
    public void setVBus(double vBus) {
        setVoltage(vBus * RobotController.getBatteryVoltage());
    }
}
