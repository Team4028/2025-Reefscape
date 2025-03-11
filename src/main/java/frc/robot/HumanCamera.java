package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class HumanCamera {
    private final HttpCamera scoreCam, climbCam;
    private final VideoSink humanCam;

    public HumanCamera() {
        scoreCam = new HttpCamera("Scoring Camera", Constants.Streams.SCORE_CAMERA, HttpCameraKind.kMJPGStreamer);
        climbCam = new HttpCamera("Climbing Camera", Constants.Streams.CLIMB_CAMERA, HttpCameraKind.kMJPGStreamer);

        humanCam = CameraServer.addSwitchedCamera("Human Camera");
        humanCam.setSource(scoreCam);
    }

    public void setCamera(boolean climb) {
        humanCam.setSource(climb ? climbCam : scoreCam);
    }

    public Command setCameraCommand(BooleanSupplier climb) {
        return Commands.runOnce(() -> setCamera(climb.getAsBoolean()));
    }
}
