package frc.robot.subsystems.limelight;

import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;
import frc.robot.util.MathUtils;

public class LimelightIO {

    private final boolean is4;
    private final String limelightName;
    private Transform3d robotToCamera;

    public LimelightIO(String limelightName, boolean is4, Optional<Integer[]> tagFilter) {
        this.is4 = is4;
        this.limelightName = limelightName;
        if (tagFilter.isPresent()) {
            LimelightHelpers.SetFiducialIDFiltersOverride(limelightName,
                    Arrays.stream(tagFilter.get()).mapToInt(Integer::intValue).toArray());
        }

        robotToCamera = new Transform3d(new Pose3d(), LimelightHelpers.getCameraPose3d_RobotSpace(limelightName));

        new Thread(() -> {
            while (robotToCamera.equals(new Transform3d())) {
                System.out.println(limelightName + " is looking for robotToCamera transform...");
                robotToCamera = new Transform3d(new Pose3d(),
                        LimelightHelpers.getCameraPose3d_RobotSpace(limelightName));
                try {
                    Thread.sleep(20);
                } catch (InterruptedException ignored) {
                }
            }
            System.out.println(limelightName + " found robotToCamera transform");
        }).start();
    }

    @AutoLog
    public static class LimelightIOInputs {
        public boolean tv = false;
        public int tid = 0;
        public double tx = 0.0;
        public double txnc = 0.0;
        public double tync = 0.0;
        public double ty = 0.0;
        public double ta = 0.0;
        public int targetCount = 0;
        public LoggablePoseEstimate solverPoseBlue = LoggablePoseEstimate.empty();
        public double[] targetPoseCameraSpace = new double[0];
        public LoggableRawFiducial[] rawFiducials = new LoggableRawFiducial[0];
        public double robotYawInternalIMU = 0;
        public int imuMode = 0;
    }

    public static final record LoggableRawFiducial(int id, double txnc, double tync, double ta, double distToCamera,
            double distToRobot, double ambiguity) {
        public static LoggableRawFiducial empty() {
            return new LoggableRawFiducial(0, 0, 0, 0, 0, 0, 0);
        }

        public static LoggableRawFiducial fromRawFiducial(RawFiducial rf) {
            return new LoggableRawFiducial(rf.id, rf.txnc, rf.tync, rf.ta, rf.distToCamera, rf.distToRobot,
                    rf.ambiguity);
        }
    }

    public static final record LoggablePoseEstimate(Pose2d pose, double timestampSeconds, double latency, int tagCount,
            double tagSpan, double avgTagDist, double avgTagArea, LoggableRawFiducial[] rawFiducials, boolean isMT2) {
        public static LoggablePoseEstimate empty() {
            return new LoggablePoseEstimate(new Pose2d(), 0, 0, 0, 0, 0, 0, null, false);
        }

        public static LoggablePoseEstimate fromPoseEstimate(PoseEstimate pe) {
            return new LoggablePoseEstimate(pe.pose, pe.timestampSeconds, pe.latency, pe.tagCount, pe.tagSpan,
                    pe.avgTagDist, pe.avgTagArea,
                    Arrays.stream(pe.rawFiducials).map(LoggableRawFiducial::fromRawFiducial)
                            .toArray(LoggableRawFiducial[]::new),
                    pe.isMegaTag2);
        }
    }

    public void updateInputs(LimelightIOInputs inputs) {
        if (LimelightHelpers.getTV(limelightName) && LimelightHelpers.getRawFiducials(limelightName).length != 0) {
            inputs.ta = LimelightHelpers.getTA(limelightName);
            inputs.targetCount = LimelightHelpers.getTargetCount(limelightName);
            inputs.solverPoseBlue = LoggablePoseEstimate
                    .fromPoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName));
            inputs.targetPoseCameraSpace = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
            try {
                inputs.tid = LimelightHelpers.getRawFiducials(limelightName)[0].id;
            } catch (ArrayIndexOutOfBoundsException ignored) {
            }
            inputs.targetCount = LimelightHelpers.getTargetCount(limelightName);
            inputs.rawFiducials = Arrays.stream(LimelightHelpers.getRawFiducials(limelightName))
                    .map(LoggableRawFiducial::fromRawFiducial).toArray(LoggableRawFiducial[]::new);
            inputs.tv = true;
            var txty = MathUtils.rotateVector(
                    new double[] { LimelightHelpers.getTX(limelightName), LimelightHelpers.getTY(limelightName) },
                    robotToCamera.getRotation().getX());
            inputs.tx = txty[0];
            inputs.ty = txty[1];
            txty = MathUtils.rotateVector(
                    new double[] { LimelightHelpers.getTXNC(limelightName), LimelightHelpers.getTYNC(limelightName) },
                    robotToCamera.getRotation().getX());
            inputs.txnc = txty[0];
            inputs.tync = txty[1];
        } else {
            inputs.tv = false;
        }
        inputs.robotYawInternalIMU = LimelightHelpers.getIMUData(limelightName).robotYaw;
        inputs.imuMode = (int) LimelightHelpers.getLimelightNTDouble(limelightName, "imumode_set");
    }

    public void seedSolverYaw(double yaw) {
        LimelightHelpers.SetRobotOrientation(limelightName, yaw, 0, 0, 0, 0, 0);
    }

    public boolean is4() {
        return is4;
    }

    public boolean setIMUInternal(boolean on) {
        if (is4) {
            LimelightHelpers.SetIMUMode(limelightName, on ? 2 : 1);
            return true;
        }

        return false;
    }

    public double getSeedAngle() {
        var e = LimelightHelpers.getLimelightDoubleArrayEntry(limelightName, "robot_orientation_set");
        if (e.exists())
            return e.get()[0];
        else return 0;
    }

    public String getName() {
        return limelightName;
    }

    public Transform3d getRobotToCamera() {
        return robotToCamera;
    }
}