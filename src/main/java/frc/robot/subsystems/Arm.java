package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorData;
import frc.robot.util.MutableElevatorFeedforward;

public class Arm extends SubsystemBase {
    private final TalonFX motor;
    private boolean hasAlgae;
    private final Elevator parentElevator;
    private final Slot0Configs pidConfig;
    private final PositionVoltage pid;
    private final ProfiledPIDController pidController;
    private final MutableElevatorFeedforward armFF;
    private double targetVBus, targetPositionRad;

   private final DigitalInput di;
   private final DutyCycleEncoder encoder;

    private double lastPosition = 0, encoderVelocity = 0;

    private ArmStates state;

    public Arm(Elevator elevator) {
        motor = new TalonFX(0);
        di = new DigitalInput(0);
        encoder = new DutyCycleEncoder(di);
        pidController = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0, 0));
        armFF = new MutableElevatorFeedforward(0, 0, 0, 0);
        hasAlgae = false;
        parentElevator = elevator;
        pid = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
        // use Elevator_Static because angle is already compensated for
        pidConfig = new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0)
                .withGravityType(GravityTypeValue.Elevator_Static);
        state = ArmStates.OFF;
    }

    private static final class ArmConstants {
        public static final double ARM_LENGTH_METRES = 0;
        public static final double ARM_MASS_KG = 0;
        public static final double CG = ARM_LENGTH_METRES / 2; // uniform density
        public static final double GEAR_RATIO = 0.03333333333333;
        public static final MotorData motorType = MotorData.KRAKEN_X60_FOC;
    }

    public static enum ArmStates {
        OFF,
        FORWARD,
        BACK,
        POSITION
    }

    public Command runMotorCommand(double vbus) {
        return runOnce(() -> {
            state = vbus == 0 ? ArmStates.OFF : vbus > 0 ? ArmStates.FORWARD : ArmStates.BACK;
            targetVBus = vbus;
        });
    }

    public Command runToPositionCommand(double positionRad) {
        return runOnce(() -> {
            state = ArmStates.POSITION;
            targetPositionRad = positionRad;
        });
    }

    /**
     * Find the torque due to gravity the arm applies on the pivot shaft
     * 
     * @param hasAlgae Whether the arm has algae in it
     * @param angleRad the angle of the arm
     * @return The arm torque (Nm)
     */
    private double armTorqueGravityNM(boolean hasAlgae, double angleRad) {
        // Arm torque assuming no algae and uniform density
        // where L = arm length, m = arm mass, g = apparent gravitational acceleration
        // (g + elevator accel)
        // T = (1/2)L * mg * sin(armAngle)
        // use effective gravity to compensate for elevator carriage acceleration
        double baseTau = ArmConstants.CG * ArmConstants.ARM_MASS_KG * (9.80665 + parentElevator.getAccelaration())
                * Math.sin(angleRad);
        if (hasAlgae) {
            // Arm torque with algae
            // r_a is radius of algae (8.125 in but in metres), m_a is mass of algae (1.5
            // lbs but in metres)
            // T_b + (L + r_a) m_a * g * sin(armAngle)
            // Assume algae is point mass that is radius of algae away from end of arm
            // Value for weight of algae is approximate due to no explicit size in manual
            // (~1.5 lbs per this post:
            // https://www.chiefdelphi.com/t/reefscape-rule-questions/478346/88?u=7dblackhole)
            return baseTau + ((ArmConstants.ARM_LENGTH_METRES + Constants.ALGAE_RADIUS_M)
                    * (9.80665 + parentElevator.getAccelaration())
                    * Constants.ALGAE_WEIGHT_KG * Math.sin(angleRad));
        }

        return baseTau;
    }

    /**
     * <p>
     * Get the voltage required to counteract the gravitational torque applied on
     * the motor shaft.
     * </p>
     * <p>
     * Used to calculate the pid controller kG gain
     * </p>
     * 
     * @return the voltage (volts)
     */
    private double armGravityFF() {
        return ArmConstants.motorType.getVoltage(
                armTorqueGravityNM(hasAlgae, encoder.get() * 2 * Math.PI) * ArmConstants.GEAR_RATIO,
                motor.getVelocity().getValueAsDouble() * 2 * Math.PI);
    }

    @Override
    public void periodic() {
        // motor.getConfigurator().apply(pidConfig.withKG(armGravityFF()));
        // armFF.setKg(armGravityFF());
        

        switch (state) {
            case OFF:
                motor.set(0);
                break;
            case FORWARD:
            case BACK:
                motor.set(targetVBus);
                break;
            case POSITION:
                motor.setVoltage(pidController.calculate(encoder.get(), targetPositionRad) + armFF.calculate(pidController.getSetpoint().velocity));
                break;
            default:
                break;
        }

        encoderVelocity = (encoder.get() - lastPosition) / 0.02;
        lastPosition = encoder.get();
    }
}
