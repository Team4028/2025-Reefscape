package frc.robot.util;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public record PIDStruct(double kP, double kI, double kD, double maxVel, double maxAccel, double maxJerk,
        double maxTourqueReverse, double maxTourqueForward, double kS, double kG, double kV, double kA) {
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

    public MotionMagicConfigs makeMMConfigs() {
        return new MotionMagicConfigs().withMotionMagicCruiseVelocity(maxVel).withMotionMagicAcceleration(maxAccel)
                .withMotionMagicJerk(maxJerk);
    }

    public TorqueCurrentConfigs makeTCConfigs() {
        return new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(maxTourqueForward)
                .withPeakReverseTorqueCurrent(maxTourqueReverse);
    }

    public Slot0Configs makeSlotConfigs(GravityTypeValue gravityType, StaticFeedforwardSignValue staticFFSign) {
        return new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKG(kG).withGravityType(gravityType)
                .withKV(kV).withKA(kA).withStaticFeedforwardSign(staticFFSign);
    }

    public Slot0Configs makeSlotConfigs(GravityTypeValue gravityType) {
        return new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKG(kG).withGravityType(gravityType)
                .withKV(kV).withKA(kA);
    }

    public Slot0Configs makeSlotConfigs() {
        return new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKG(kG).withKV(kV).withKA(kA);
    }
}
