package frc.robot.subsystems.groundinfeed;

import com.playingwithfusion.TimeOfFlight;

public class GrondTOFIOPWF implements GrondTOFIO {
    private final TimeOfFlight tof = new TimeOfFlight(GrondConstants.PWFTimeOfFlight.CAN_ID);

    public GrondTOFIOPWF() {
        tof.setRangingMode(GrondConstants.PWFTimeOfFlight.mode,
                GrondConstants.PWFTimeOfFlight.sampleTime.orElse(tof.getSampleTime()));
        tof.setRangeOfInterest(0, 0, 15, 15);
    }

    @Override
    public void updateInputs(GrondTOFIOInputs inputs) {
        inputs.lightingLevel = tof.getAmbientLightLevel();
        inputs.range = tof.getRange();
        inputs.rangeSigma = tof.getRangeSigma();
        inputs.rangeValid = tof.isRangeValid();
        inputs.samplingTime = tof.getSampleTime();
    }

    @Override
    public void setRangeOfI(int topX, int topY, int bottomX, int bottomY) {
        tof.setRangeOfInterest(topX, topY, bottomX, bottomY);
    }
}
