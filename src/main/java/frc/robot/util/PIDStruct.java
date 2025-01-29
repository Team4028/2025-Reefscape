package frc.robot.util;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public record PIDStruct(double kP, double kI, double kD, double maxVel, double maxAccel, double kS, double kG, double kV, double kA)
{
    public PIDController makeController() {
        return new PIDController(kP, kI, kD);
    }

    public ProfiledPIDController makeProfiledPIDController() {
        return new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVel, maxAccel));
    }

    public ElevatorFeedforward makeElevatorFeedforward() {
        return new ElevatorFeedforward(kS, kG, kV, kA);
    }

    public ArmFeedforward makeArmFeedforward() {
        return new ArmFeedforward(kS, kG, kV, kA);
    }
}
