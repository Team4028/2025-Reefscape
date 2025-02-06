package frc.robot.subsystems.arm;

import frc.robot.Constants;

public class ArmPhysics {
        /**
     * Find the torque due to gravity the arm applies on the pivot shaft
     * 
     * @param hasAlgae Whether the arm has algae in it
     * @param angleRad the angle of the arm
     * @return The arm torque (Nm)
     */
    private static double armTorqueGravityNM(boolean hasAlgae, double angleRad, double elevatorAcceleration) {
        // Arm torque assuming no algae and uniform density
        // where L = arm length, m = arm mass, g = apparent gravitational acceleration
        // (g + elevator accel)
        // T = (1/2)L * mg * sin(armAngle)
        // use effective gravity to compensate for elevator carriage acceleration
        double baseTau = ArmConstants.CG * ArmConstants.ARM_MASS_Kg * (9.80665 + elevatorAcceleration)
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
                    * (9.80665 + elevatorAcceleration)
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
    public static double armGravityFF(boolean hasAlgae, double armPositionRad, double motorVelocity, double elevatorAccelaration) {
        return ArmConstants.Sim.simGearbox.getVoltage(
                armTorqueGravityNM(hasAlgae, armPositionRad, elevatorAccelaration) * ArmConstants.GEAR_RATIO,
                motorVelocity * ArmConstants.PI_2);
    }
}
