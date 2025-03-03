package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.RobotSim;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim arm;
    private double targetVolts = 0.0;
    private final ProfiledPIDController pid;
    private final ArmFeedforward armFF;

    public ArmIOSim() {
        arm = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(ArmConstants.Sim.simGearbox,
                        ArmConstants.Sim.ARM_MOI_KgMSquared,
                        ArmConstants.GEAR_RATIO),
                ArmConstants.Sim.simGearbox, ArmConstants.GEAR_RATIO, ArmConstants.ARM_LENGTH_METRES,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                true, 0);
        RobotSim.registerCurrentInput("Arm", arm::getCurrentDrawAmps);
        pid = ArmConstants.simPidConfig.makeProfiledPIDController();
        armFF = ArmConstants.simPidConfig.makeArmFeedforward();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        arm.setInput(targetVolts);
        arm.update(0.02);
        inputs.appliedVoltage = targetVolts;
        inputs.armAngleRad = arm.getOutput(0);
        inputs.armEncoderRad = arm.getOutput(0) + ArmConstants.PI_3_2;
        inputs.armEncoderRaw = arm.getOutput(0) / ArmConstants.PI_2 + 0.25;
        inputs.armVelocityRotPerSec = arm.getOutput(1) / ArmConstants.PI_2;
        inputs.armMotorVelocityRotPerSec = inputs.armVelocityRotPerSec / ArmConstants.GEAR_RATIO;
        inputs.currentAmps = arm.getCurrentDrawAmps();
        ArmIO.super.updateInputs(inputs);
    }

    @Override
    public void setPID(double position) {
        setVoltage(pid.calculate(arm.getOutput(0) + ArmConstants.PI_3_2, position)
                + armFF.calculate(arm.getOutput(0), pid.getSetpoint().velocity));
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
