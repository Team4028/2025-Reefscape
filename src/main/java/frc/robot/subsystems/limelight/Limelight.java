package frc.robot.subsystems.limelight;

import static edu.wpi.first.units.Units.Radians;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.limelight.LimelightIO.LoggablePoseEstimate;
import frc.robot.util.VisionUtil;

public class Limelight extends SubsystemBase {
    private final LimelightIO io;
    private final LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();
    private final String name;

    public Limelight(LimelightIO io) {
        this.io = io;
        name = io.getName();
        VisionUtil.registerPoseSource(this, this::getBotposeEstimateMT2);
    }

    public boolean trustPose(Translation2d driveTrans) {
        return driveTrans.getDistance(
                inputs.solverPoseBlue.pose().getTranslation()) <= LimelightConstants.STD_DEV_POSE_DIFF_THRESHOLD;
    }

    public double getTX() {
        return inputs.tx;
    }

    public double getTY() {
        return inputs.ty;
    }

    public double getTA() {
        return inputs.ta;
    }

    public double getTA(int tagID) {
        for (var rf : inputs.rawFiducials)
            if (rf.id() == tagID)
                return rf.ta();
        return Double.NaN;
    }

    public double getTXNC() {
        return inputs.txnc;
    }

    public double getTXNC(int tagID) {
        for (var rf : inputs.rawFiducials)
            if (rf.id() == tagID)
                return rf.txnc();
        return Double.NaN;
    }

    public double getTYNC() {
        return inputs.tync;
    }

    public double getTYNC(int tagID) {
        for (var rf : inputs.rawFiducials)
            if (rf.id() == tagID)
                return rf.tync();
        return Double.NaN;
    }

    public int getTargetCount() {
        return inputs.targetCount;
    }

    public int getTagID() {
        return inputs.tid;
    }

    public double[] getTargetPoseCameraSpace() {
        return inputs.targetPoseCameraSpace;
    }

    public boolean getTV() {
        return inputs.tv;
    }

    public LoggablePoseEstimate getBotposeEstimateMT2(double driveYawRad) {
        var vRes = inputs.solverPoseBlue;
        if (vRes.tagCount() > 1)
            return vRes;

        if (vRes.tagCount() < 1)
            return LoggablePoseEstimate.empty();

        // Mechanical Advantage shenanigens
        // https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/85
        int tagID = vRes.rawFiducials()[0].id();
        double tync = vRes.rawFiducials()[0].tync();
        double txnc = vRes.rawFiducials()[0].txnc();
        Pose3d tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(tagID).get();
        double dist2d = Math
                .cos(io.getRobotToCamera().getRotation().getMeasureY().in(Radians) + Units.degreesToRadians(tync))
                * vRes.rawFiducials()[0].distToCamera();
        var dYawRad = io.getRobotToCamera().getRotation().getMeasureZ().in(Radians) + Units.degreesToRadians(txnc);
        return new LoggablePoseEstimate(
                new Pose2d(tagPose.toPose2d().getTranslation().minus(new Translation2d(Math.cos(dYawRad) * dist2d,
                        Math.sin(dYawRad) * dist2d)).plus(io.getRobotToCamera().getTranslation().toTranslation2d()),
                        Rotation2d.fromRadians(driveYawRad)),
                vRes.timestampSeconds(), vRes.latency(), 1, vRes.tagSpan(), dist2d, vRes.avgTagArea(),
                vRes.rawFiducials(), vRes.isMT2());
    }

    public boolean setIMUInternal(boolean on) {
        return io.setIMUInternal(on);
    }

    public void seedLLSolverYaw(double chassisYaw) {
        io.seedSolverYaw(chassisYaw);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision/" + name, inputs);
    }
}
