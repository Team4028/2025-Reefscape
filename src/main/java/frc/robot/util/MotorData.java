package frc.robot.util;

public enum MotorData {
    KRAKEN_X60(7.09, 366),
    KRAKEN_X60_FOC(9.37, 483),
    KRAKEN_X44(4.05, 275),
    FALCON_500(4.69, 257),
    FALCON_500_FOC(5.84, 304),
    NEO_VORTEX(3.6, 211),
    NEO(2.6, 105),
    NEO_550(0.97, 100),
    PRO_775(0.71, 134);

    public double stallTorque;
    public double stallCurrent;

    private MotorData(double stallTorque, double stallCurrent) {
        this.stallTorque = stallTorque;
        this.stallCurrent = stallCurrent;
    }

    /**
     * Gets the motor's torque constant
     */
    public double kt() {
        return stallTorque / stallCurrent;
    }

    /**
     * Gets the back EMF of the motor 
     * @param angVelRadPS the current angular velocity of the motor (rad/s)
     * @return the back emf (volts)
     */
    public double backEMF(double angVelRadPS) {
        return kt() * angVelRadPS;
    }

    /**
     * Gets the resitance of the motor
     * @param voltage the current voltage applied to the motor (volts)
     * @return the resistance (ohms)
     */
    public double resistance(double voltage) {
        return voltage / stallCurrent;
    }

    /**
     * Gets the output torque of the motor
     * @param voltage the current voltage applied to the motor (volts)
     * @param angVelRadPS the current angular velocity of the motor (rad/s)
     * @return the torque (Nm)
     */
    public double getTorque(double voltage, double angVelRadPS) {
        return kt() * ((voltage - backEMF(angVelRadPS)) / resistance(voltage));
    }

    /**
     * Gets the voltage of the motor required to produce the output torque at the current speed
     * @param torque the motor output torque (Nm)
     * @param angVelRadPS the current motor veolcity (rad/s)
     * @return the voltage (volts)
     */
    public double getVoltage(double torque, double angVelRadPS) {
        return ((Math.pow(kt(), 2) * angVelRadPS) / (kt() - torque / stallCurrent));
    }

    /**
     * Gets the angular velocity at which the applied voltage results in the output torque
     * @param torque the output torque (Nm)
     * @param voltage the input voltage (volts)
     * @return the angular veolcity (rad/s)
     */
    public double getAngularVelocity(double torque, double voltage) {
        return ((kt() * voltage - torque * voltage / stallCurrent) / Math.pow(kt(), 2));
    }
}
