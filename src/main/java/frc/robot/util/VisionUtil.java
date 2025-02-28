package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleFunction;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.limelight.LimelightIO.LoggablePoseEstimate;

public class VisionUtil {
    public static Map<Limelight, DoubleFunction<LoggablePoseEstimate>> poseSources = new HashMap<>();

    private VisionUtil() {}

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
}