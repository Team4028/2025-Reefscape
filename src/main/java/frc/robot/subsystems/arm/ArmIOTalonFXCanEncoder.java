package frc.robot.subsystems.arm;

import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagFaults;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

public class ArmIOTalonFXCanEncoder implements ArmIO {
    private final Canandmag canMag;
    private CanandmagSettings settings;
    CanandmagFaults faults;

    double position, velocity;

    public ArmIOTalonFXCanEncoder() {
        /*
         * You should be able to just use the position of the canmag in arm to tell when
         * to stop or when to go.
         */
        canMag = new Canandmag(10);
        settings = new CanandmagSettings();

        faults = canMag.getActiveFaults();

        // sets velocity width settings to 25
        settings.setVelocityFilterWidth(25);

        settings.setInvertDirection(false);
        settings.setDisableZeroButton(false);

        position = canMag.getPosition();
        velocity = canMag.getVelocity();

        /*
         * May use:
         * settings.setPositionFramePeriod(0.020);
         * settings.setVelocityFramePeriod(0.020);
         * settings.setStatusFramePeriod(1);
         */

        // Fun light stuff
        canMag.setPartyMode(50);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.canMagPosition = canMag.getPosition();
        inputs.canMagVelocity = canMag.getVelocity();
        inputs.canMagInRange = canMag.magnetInRange();
        ArmIO.super.updateInputs(inputs);
    }

    public void setPosition(double newPosition) {
        // This just sets the position of the canMag to whatever you say it is. This
        // does not run the mag to the position.
        canMag.setPosition(newPosition);
    }

    public void doSomethingAfterInRange() {
        if (canMag.magnetInRange()) {
            emptyMethod();
        }
    }

    public void emptyMethod() {

    }

    public void zeroCanMag() {
        canMag.zeroAll();
    }

    // possible uses for the future:
    // Set position: canandmag.setPosition(100);
    // Set Abs position: canandmag.setAbsPosition(0.5);
    // Offsets: canandmag.setAbsPosition(0.3, 0, false);
    // Zero: canandmag.zeroAll();
}
