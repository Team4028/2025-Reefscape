package frc.robot.subsystems.arm;


import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagFaults;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;



public class ArmIOTalonFXCanEncoder implements ArmIO {
    private final Canandmag canMag;
    private CanandmagSettings settings;
    CanandmagFaults faults;

    public ArmIOTalonFXCanEncoder() {
        canMag = new Canandmag(10);
        settings = new CanandmagSettings();

        faults = canMag.getActiveFaults();

        //sets velocity width settings to 25
        settings.setVelocityFilterWidth(25);


        settings.setInvertDirection(false);
        settings.setDisableZeroButton(false);

        /* May use: 
         * settings.setPositionFramePeriod(0.020);
         * settings.setVelocityFramePeriod(0.020);
         * settings.setStatusFramePeriod(1);
         */

         //Fun light stuff
         canMag.setPartyMode(50);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.canMagPosition = canMag.getPosition();
        inputs.canMagVelocity = canMag.getVelocity();
        ArmIO.super.updateInputs(inputs);
    }


    //possible uses for the future:
    //Set position: canandmag.setPosition(100);
    //Set Abs position: canandmag.setAbsPosition(0.5);
    //Offsets: canandmag.setAbsPosition(0.3, 0, false);
    //Zero: canandmag.zeroAll();
}
