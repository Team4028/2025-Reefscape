package frc.robot.subsystems.limelight;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.limelight.LimelightIO.LoggablePoseEstimate;
import frc.robot.util.VisionUtil;

public class Limelight extends SubsystemBase {
    private final LimelightIO io;
    private final LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();
    private final String name;
    private static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape); // 6-11; 17-22

    public Limelight(LimelightIO io) {
        this.io = io;
        name = io.getName();
        VisionUtil.registerPoseSource(this, this::getBotposeEstimateMT2);
    }

    public boolean trustPose(Translation2d driveTrans) {
        return getTV();
        //driveTrans.getDistance(
          //      inputs.solverPoseBlue.pose().getTranslation()) <= LimelightConstants.STD_DEV_POSE_DIFF_THRESHOLD;
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

    @Override
    public String getName() {
        return io.getName();
    }
     
    public LoggablePoseEstimate getBotposeEstimateMT2(double driveYawRad) {
        var vRes = inputs.solverPoseBlue;
        if (vRes.tagCount() > 1) 
            return vRes;

        if (vRes.tagCount() < 1 || !getTV())
            return LoggablePoseEstimate.empty();

        // Mechanical Advantage shenanigens
        // https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/85
        int tagID = vRes.rawFiducials()[0].id();
        double tync = vRes.rawFiducials()[0].tync();
        double txnc = vRes.rawFiducials()[0].txnc();
        Pose2d tagPose2d = field.getTagPose(tagID).get().toPose2d();
        Translation2d camToTagTranslation = 
            new Pose3d(Translation3d.kZero, new Rotation3d(0, Units.degreesToRadians(tync), Units.degreesToRadians(-txnc)))
                .transformBy(
                    new Transform3d(new Translation3d(vRes.rawFiducials()[0].distToCamera(), 0, 0),
                        Rotation3d.kZero))
                .getTranslation()
                .rotateBy(new Rotation3d(0, io.getRobotToCamera().getRotation().getY(), 0))
                .toTranslation2d();

        double camToTagRotation = 
            driveYawRad + io.getRobotToCamera().getRotation().getMeasureZ().in(Radians)
                - camToTagTranslation.getAngle().getRadians();

        Translation2d fieldToCameraTranslation = 
            new Pose2d(tagPose2d.getTranslation(),
                Rotation2d.fromRadians(camToTagRotation - Math.PI/2))
                .transformBy(new Transform2d(camToTagTranslation.getNorm(), 0, Rotation2d.kZero))
                .getTranslation();

        Pose2d robotPose = 
            new Pose2d(fieldToCameraTranslation,
                Rotation2d.fromRadians(driveYawRad - io.getRobotToCamera().getRotation().getZ()))
                .transformBy(new Transform2d(new Pose2d(-io.getRobotToCamera().getX(), -io.getRobotToCamera().getY(),
                        io.getRobotToCamera().getRotation().toRotation2d()), Pose2d.kZero));

        // Debug logs
        // Logger.recordOutput("bp/Fiducial", vRes.rawFiducials()[0]);
        // Logger.recordOutput("bp/Cam to Tag Tran", camToTagTranslation);
        // Logger.recordOutput("bp/Cam to Tag Rot", camToTagRotation);
        // Logger.recordOutput("bp/Feid to Cam", fieldToCameraTranslation);
        // Logger.recordOutput("bp/Robot Pose", robotPose);

        return new LoggablePoseEstimate(new Pose2d(robotPose.getTranslation(), Rotation2d.fromRadians(driveYawRad)),
                vRes.timestampSeconds(), vRes.latency(), vRes.tagCount(), vRes.tagSpan(), vRes.avgTagDist(),
                vRes.avgTagArea(), vRes.rawFiducials(), vRes.isMT2());

    }

    public void warmup() {
        periodic();
        getBotposeEstimateMT2(0);
        setIMUInternal(false);
        seedLLSolverYaw(0);
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
