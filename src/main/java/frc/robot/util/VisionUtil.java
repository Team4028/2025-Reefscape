package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleFunction;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.limelight.LimelightIO.LoggablePoseEstimate;

public class VisionUtil {
    public static Map<Limelight, DoubleFunction<LoggablePoseEstimate>> poseSources = new HashMap<>();

    private VisionUtil() {
    }

    public static void registerPoseSource(Limelight ll, DoubleFunction<LoggablePoseEstimate> source) {
        poseSources.put(ll, source);
    }

    public static Map<Limelight, DoubleFunction<LoggablePoseEstimate>> poseSources() {
        return poseSources;
    }

    public static Limelight[] registeredLimelights() {
        return poseSources.keySet().toArray(Limelight[]::new);
    }

    public static DoubleFunction<LoggablePoseEstimate> removeSource(Limelight ll) {
        return poseSources.remove(ll);
    }

    public static void clearSources() {
        poseSources.clear();
    }

    public static void logPoses(Drive drivetrain) {
        poseSources.entrySet().forEach(e -> {
            Logger.recordOutput("Vision/" + e.getKey().getName() + "/EstPose",
                    e.getValue().apply(drivetrain.getPose().getRotation().getRadians()));
        });
    }

    public static LoggablePoseEstimate getPoseFromSource(Limelight ll, double driveYawRad) {
        return poseSources.get(ll).apply(driveYawRad);
    }

    public static void addMeasurements(Consumer<LoggablePoseEstimate> poseAdder, Drive drivetrain) {
        poseSources.entrySet().stream().filter(e -> e.getKey().trustPose(drivetrain.getPose().getTranslation()))
                .forEach(e -> poseAdder.accept(e.getValue().apply(drivetrain.getPose().getRotation().getRadians())));
    }

    public static Command addMeasurementsCommand(Consumer<LoggablePoseEstimate> poseAddr, Drive drivetrain) {
        return Commands.runOnce(() -> addMeasurements(poseAddr, drivetrain));
    }

    public static final class LimelightSim {
        private static final VisionSystemSim vSim = new VisionSystemSim("main");
        private static final SimCameraProperties llProp = new SimCameraProperties();
        private final PhotonCamera cam;
        private final PhotonCameraSim sim;
        private final String name;

        static {
            vSim.addAprilTags(Limelight.field);
            llProp.setCalibration(1280, 960, Rotation2d.fromDegrees(91.14));
            llProp.setCalibError(0.25, 0.08);
            llProp.setFPS(40);
            llProp.setAvgLatencyMs(10);
            llProp.setLatencyStdDevMs(5);
        }

        public LimelightSim(Limelight ll, Transform3d robotToCameraTransform) {
            name = ll.getName() + "_sim";
            cam = new PhotonCamera(name);
            sim = new PhotonCameraSim(cam, llProp);
            vSim.addCamera(sim, robotToCameraTransform);
            sim.enableRawStream(true);
            sim.enableProcessedStream(true);
            sim.enableDrawWireframe(true);
        }

        public String getName() {
            return name;
        }

        public void updateRobotPose(Pose2d drivePose) {
            vSim.update(drivePose);
        }

        public Pose3d[] getTagsSeen() {
            ArrayList<Pose3d> poses = new ArrayList<>();
            cam.getAllUnreadResults().stream().forEach(r -> poses.addAll(r.targets.stream()
                    .map(t -> Limelight.field.getTagPose(t.fiducialId).orElse(new Pose3d())).toList()));
            return poses.toArray(Pose3d[]::new);
        }
    }
}