package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorData;

public class Arm extends SubsystemBase {
    private final TalonFX motor;
    private boolean hasAlgae;
    private final Elevator parentElevator;
    private final Slot0Configs pidConfig;
    private final PositionVoltage pid;
    private double targetVBus, targetPositionRad;

    private ArmStates state;

    public Arm(Elevator elevator) {
        motor = new TalonFX(0);
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
        return ArmConstants.motorType.getVoltage(armTorqueGravityNM(hasAlgae, motor.getPosition().getValueAsDouble()),
                motor.getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic() {
        motor.getConfigurator().apply(pidConfig.withKG(armGravityFF()));

        switch (state) {
            case OFF:
                motor.set(0);
                break;
            case FORWARD:
            case BACK:
                motor.set(targetVBus);
                break;
            case POSITION:
                motor.setControl(pid.withPosition(targetPositionRad));
                break;
            default:
                break;
        }
    }
}
