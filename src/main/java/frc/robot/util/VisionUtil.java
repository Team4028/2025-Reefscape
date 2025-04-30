package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleFunction;

import lombok.Getter;
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
import lombok.experimental.UtilityClass;

@UtilityClass
public class VisionUtil {
    public static Map<Limelight, DoubleFunction<LoggablePoseEstimate>> poseSources = new HashMap<>();
    public static List<LimelightSim> sims = new ArrayList<>();
    public static boolean requestingSeed = false;

    public static void registerPoseSource(Limelight ll, DoubleFunction<LoggablePoseEstimate> source) {
        poseSources.put(ll, source);
    }

    public static Map<Limelight, DoubleFunction<LoggablePoseEstimate>> poseSources() {
        return poseSources;
    }

    public static void seedIMUs(double driveDeg) {
        poseSources.keySet().forEach(l -> l.seedLLSolverYaw(driveDeg));
    }

    public static void setLLIMUModes(boolean on) {
        poseSources.keySet().forEach(l -> l.setIMUInternal(on));
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
        poseSources.forEach((key, value) -> Logger.recordOutput("Vision/" + key.getName() + "/EstPose",
                value.apply(drivetrain.getPose().getRotation().getRadians())));
    }

    public static LoggablePoseEstimate getPoseFromSource(Limelight ll, double driveYawRad) {
        return poseSources.get(ll).apply(driveYawRad);
    }

    public static void addMeasurements(Consumer<LoggablePoseEstimate> poseAdder, Drive drivetrain) {
        for (var e : poseSources.entrySet()) {
            Pose2d dtPose = drivetrain.getPose();
            if (!e.getKey().trustPose(dtPose.getTranslation()))
                continue;
            poseAdder.accept(e.getValue().apply(dtPose.getRotation().getRadians()));
        }
    }

    public static Command addMeasurementsCommand(Consumer<LoggablePoseEstimate> poseAddr, Drive drivetrain) {
        return Commands.runOnce(() -> addMeasurements(poseAddr, drivetrain));
    }

    public static void bindSimCameras(Transform3d[] robotToCameraTransforms) {
        for (int i = 0; i < robotToCameraTransforms.length; i++) {
            sims.add(new LimelightSim(new ArrayList<>(poseSources.keySet()).get(i).getName(), robotToCameraTransforms[i]));
        }
    }

    public static void updateSimDrivePose(Pose2d drivePose) {
        sims.get(0).updateRobotPose(drivePose);
    }

    public static void logSeenTags() {
        sims.forEach(s -> Logger.recordOutput("Vision/" + s.getName() + "/TagPoses", s.getTagsSeen()));
    }

    public static final class LimelightSim {
        private static final VisionSystemSim vSim = new VisionSystemSim("main");
        private static final SimCameraProperties llProp = new SimCameraProperties();
        private final PhotonCamera cam;
        @Getter
        private final String name;

        static {
            vSim.addAprilTags(Limelight.field);
            llProp.setCalibration(1280, 960, Rotation2d.fromDegrees(91.14));
            llProp.setCalibError(0.25, 0.08);
            llProp.setFPS(40);
            llProp.setAvgLatencyMs(10);
            llProp.setLatencyStdDevMs(5);
        }

        public LimelightSim(String llName, Transform3d robotToCameraTransform) {
            name = llName + "_sim";
            cam = new PhotonCamera(name);
            PhotonCameraSim sim = new PhotonCameraSim(cam, llProp);
            vSim.addCamera(sim, robotToCameraTransform);
            sim.enableRawStream(true);
            sim.enableProcessedStream(true);
            sim.enableDrawWireframe(true);
        }

        public void updateRobotPose(Pose2d drivePose) {
            vSim.update(drivePose);
        }

        public Pose3d[] getTagsSeen() {
            ArrayList<Pose3d> poses = new ArrayList<>();
            for (var r : cam.getAllUnreadResults()) {
                for (var t : r.targets) {
                    poses.add(Limelight.field.getTagPose(t.fiducialId).orElse(new Pose3d()));
                }
            }

            return poses.toArray(Pose3d[]::new);
        }
    }
}