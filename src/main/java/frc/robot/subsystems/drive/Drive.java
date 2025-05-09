package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Armistice;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.LoggedTunables.LoggedTunableNumber;
import frc.robot.util.MathUtils;
import frc.robot.util.VisionUtil;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Comparator;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

@ExtensionMethod({ MathUtils.class })
public class Drive extends SubsystemBase {
    // TunerConstants doesn't include these constants, so they are declared locally
    static final double ODOMETRY_FREQUENCY = new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD()
            ? 250.0
            : 100.0;
    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 48.5344;
    private static final double ROBOT_MOI = 6.883;
    private static final double WHEEL_COF = 1.2;
    private static final RobotConfig PP_CONFIG = new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                    TunerConstants.FrontLeft.WheelRadius,
                    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                    WHEEL_COF,
                    DCMotor.getKrakenX60Foc(1)
                            .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                    TunerConstants.FrontLeft.SlipCurrent,
                    1),
            getModuleTranslations());

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
            AlertType.kError);

    @AutoLogOutput
    private boolean useGPVisionCorrect = false;
    private Limelight gpVisionSource = null;
    private double lastTY = 0;

    private boolean isFinished2dAlign = true;
    private double filteredX, filteredY, filteredRot;
    private final LinearFilter xFilter, yFilter, rotFilter;
    private Limelight limelightLineupSource2d = null;
    private final ProfiledPIDController xPid, yPid, rotPid;
    private int ll2dLineupTagID = 0;

    private final LinearFilter txCoralFilter = LinearFilter.movingAverage(5);
    private final PIDController txCoralLockPid = new PIDController(0.05, 0, 0.005);

    private final LoggedTunableNumber branchOffsetM, coralScoreOffsetIn, algaeOffsetIn;

    public static final double PID_TRANSLATION_SPEED_MPS = 1.85;
    public static final double PID_ROTATION_RAD_PER_SEC = Math.PI;
    private static final double AUTON_PATH_CANCEL_RADIUS_M = 2;
    private static final double GP_CORRECTION_SPEED = 1;

    @AutoLogOutput(key = "Odometry/ClosestReef")
    private Pose2d closestReef = new Pose2d();

    @AutoLogOutput
    private String closestReefName = "";

    private AprilTag closestReefTag = Limelight.field.getTags().get(0);

    @Setter
    private boolean reefTargetIsRight = true;

    @AutoLogOutput
    private double pidErrorM = 0;

    @AutoLogOutput
    private double pidThetaR = 0;

    @AutoLogOutput
    private boolean inPidTranslate = false;

    private final PIDController pidLineup = new PIDController(5, 0, 0);

    private final PIDController angleController = new PIDController(8, 0, 0);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
            lastModulePositions, new Pose2d());

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {
        InterpolatingDoubleTreeMap txty = new InterpolatingDoubleTreeMap();
        txty.put(-20.0, -15.);
        txty.put(-5.2, -12.2);
        txty.put(3., -8.);
        txty.put(5.8, -7.1);
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

        branchOffsetM = new LoggedTunableNumber("Branch Offset (M)", Constants.TAG_TO_BRANCH_OFFSET_M);
        coralScoreOffsetIn = new LoggedTunableNumber("Coral Scoring Offset (In)",
                Constants.CORAL_SCORE_OFFSET_FROM_CENTERLINE_IN);
        algaeOffsetIn = new LoggedTunableNumber("Algae ACquire Offset (In)",
                Constants.ALGAE_SCORE_OFFSET_FROM_CENTERLINE_IN);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::runVelocity,
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> Logger.recordOutput(
                        "Odometry/Trajectory", activePath.toArray(new Pose2d[0])));
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

        xFilter = LinearFilter.movingAverage(5);
        yFilter = LinearFilter.movingAverage(5);
        rotFilter = LinearFilter.movingAverage(5);
        xPid = new ProfiledPIDController(.28, 0, 0, new TrapezoidProfile.Constraints(3, 6));
        yPid = new ProfiledPIDController(.8, 0, 0, new TrapezoidProfile.Constraints(3, 6));
        rotPid = new ProfiledPIDController(.06, 0, 0, new TrapezoidProfile.Constraints(90, 180));
        pidLineup.setTolerance(0.00635);
        angleController.setTolerance(Units.degreesToRadians(1));
        angleController.enableContinuousInput(0, 2 * Math.PI);

        SmartDashboard.putData("Swerve Drive", builder -> {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty("Front Left Angle", () -> getModuleStates()[0].angle.getRadians(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> getModuleStates()[0].speedMetersPerSecond, null);

            builder.addDoubleProperty("Front Right Angle", () -> getModuleStates()[0].angle.getRadians(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> getModuleStates()[0].speedMetersPerSecond,
                    null);

            builder.addDoubleProperty("Back Left Angle", () -> getModuleStates()[0].angle.getRadians(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> getModuleStates()[0].speedMetersPerSecond, null);

            builder.addDoubleProperty("Back Right Angle", () -> getModuleStates()[0].angle.getRadians(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> getModuleStates()[0].speedMetersPerSecond, null);

            builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians(), null);
        });
    }

    /**
     * Enables gamepiece vision-based driving correction (via runVelocity), or
     * disables it if the limelight is null
     */
    public void setGPVisionCorrection(Limelight limelight) {
        useGPVisionCorrect = (gpVisionSource = limelight) != null;
    }

    @Override
    public void periodic() {
        if (gpVisionSource != null) txCoralFilter.calculate(gpVisionSource.getTX());
        Logger.recordOutput("Drive/ReefSide", reefTargetIsRight ? "R" : "L");
        Logger.recordOutput("Drive/ClosestTagID", closestReefTag.ID);
        Logger.recordOutput("Current Command", getCurrentCommand() == null ? "" : getCurrentCommand().getName());
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        closestReef = closestReefPose();
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                                - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode == Mode.REAL);

        if (limelightLineupSource2d != null && !isFinished2dAlign) {
            filteredX = xFilter.calculate(limelightLineupSource2d.getTXNC(ll2dLineupTagID));
            filteredY = yFilter.calculate(limelightLineupSource2d.getTA(ll2dLineupTagID));
            filteredRot = rotFilter.calculate(limelightLineupSource2d.getTargetPoseCameraSpace()[4]);
        }
    }

    public BooleanSupplier driveCloseEnoughReefAuton() {
        return () -> getPose().getTranslation()
                .getDistance(closestReefTag.pose.toPose2d().getTranslation()) < AUTON_PATH_CANCEL_RADIUS_M;
    }

    public BooleanSupplier driveCloseEnoughAcquireAutonLeftLoli() {
        return () -> getPose().getTranslation().getDistance(
                AutoBuilder.shouldFlip()
                        ? FlippingUtil.flipFieldPosition(Constants.AQUIRE_LOLI_LEFT_POS.getTranslation())
                        : Constants.AQUIRE_LOLI_LEFT_POS.getTranslation()) < AUTON_PATH_CANCEL_RADIUS_M;
    }

    public BooleanSupplier driveCloseEnoughAcquireAutonMidLoli() {
        return () -> getPose().getTranslation().getDistance(
                AutoBuilder.shouldFlip()
                        ? FlippingUtil.flipFieldPosition(Constants.AQUIRE_LOLI_MID_POS.getTranslation())
                        : Constants.AQUIRE_LOLI_MID_POS.getTranslation()) < AUTON_PATH_CANCEL_RADIUS_M;
    }

    public BooleanSupplier driveCloseEnoughAcquireAutonRightLoli() {
        return () -> getPose().getTranslation().getDistance(
                AutoBuilder.shouldFlip()
                        ? FlippingUtil.flipFieldPosition(Constants.AQUIRE_LOLI_RIGHT_POS.getTranslation())
                        : Constants.AQUIRE_LOLI_RIGHT_POS.getTranslation()) < AUTON_PATH_CANCEL_RADIUS_M;
    }

    public BooleanSupplier driveCloseEnoughAcquireAuton() {
        return () -> getPose().getTranslation().getDistance(
                AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPosition(Constants.AQUIRE_RIGHT_POS.getTranslation())
                        : Constants.AQUIRE_RIGHT_POS.getTranslation()) < AUTON_PATH_CANCEL_RADIUS_M;
    }

    public BooleanSupplier hasCoralCorrection() {
        return () -> useGPVisionCorrect;
    }

    public void resetGPTY() {
        lastTY = 0;
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        if (!inPidTranslate && gpVisionSource != null && useGPVisionCorrect) {
            speeds.vxMetersPerSecond = MathUtils.clamp(
                            txCoralLockPid.calculate(txCoralFilter.lastValue(), 0),
                            -3.0,
                            3.0) * GP_CORRECTION_SPEED;
            if (gpVisionSource.getTV())
                lastTY = gpVisionSource.getTY();
        } else if (gpVisionSource == null) {
            lastTY = 0;
            txCoralLockPid.reset();
        }
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    public Pose2d closestReefPoseAlgae() {
        var nativePose = closestReefPose();
        var nativePoseNativeRot = new Pose2d(nativePose.getTranslation(),
                nativePose.getRotation().minus(Constants.SCORING_SIDE_FROM_FRONT_ROT));
        var invTrPose = nativePoseNativeRot
                .transformBy(new Transform2d(0,
                        ((reefTargetIsRight ? -branchOffsetM.get() : branchOffsetM.get())
                                + Units.inchesToMeters(coralScoreOffsetIn.get()))
                                - Units.inchesToMeters(algaeOffsetIn.get()),
                        Rotation2d.kZero));
        return new Pose2d(invTrPose.getTranslation(), nativePose.getRotation());
    }

    public Pose2d closestReefPose() {
        AprilTag closestTag = Limelight.field.getTags().stream()
                .filter(t -> Constants.reefTagNames.containsKey(t.ID)).min(Comparator.comparingDouble(
                        t -> t.pose.toPose2d().getTranslation().getDistance(getPose().getTranslation()))).orElse(Limelight.field.getTags().get(0));
        Pose2d closestPose = closestTag.pose.toPose2d()
                .transformBy(new Transform2d(Units.inchesToMeters(Constants.SCORING_SIDE_RADIUS_ROBOT_IN),
                        ((reefTargetIsRight ? branchOffsetM.get() : -branchOffsetM.get())
                                - Units.inchesToMeters(coralScoreOffsetIn.get())),
                        Rotation2d.kZero));
        closestReefName = Constants.reefTagNames.get(closestTag.ID);
        closestReefTag = closestTag;
        return new Pose2d(closestPose.getTranslation(),
                closestPose.getRotation().plus(Constants.SCORING_SIDE_FROM_FRONT_ROT));
    }

    @AutoLogOutput
    public Pose2d pipe1ClosestReefPose() {
        var crPose = closestReefPose();
        var crPoseNativeRot = new Pose2d(crPose.getTranslation(),
                crPose.getRotation().minus(Constants.SCORING_SIDE_FROM_FRONT_ROT));
        var plusCoral = crPoseNativeRot
                .transformBy(new Transform2d(Units.inchesToMeters(Constants.CORAL_DIAM_IN), 0, Rotation2d.kZero));
        return new Pose2d(plusCoral.getTranslation(), crPose.getRotation());
    }

    @AutoLogOutput
    public Pose2d pipe1AlgaeClosestReefPose() {
        var crPose = closestReefPoseAlgae();
        var crPoseNativeRot = new Pose2d(crPose.getTranslation(),
                crPose.getRotation().minus(Constants.SCORING_SIDE_FROM_FRONT_ROT));
        var plosCoral = crPoseNativeRot
                .transformBy(new Transform2d(Units.inchesToMeters(Constants.CORAL_DIAM_IN), 0, Rotation2d.kZero));
        return new Pose2d(plosCoral.getTranslation(), crPose.getRotation());
    }

    public Pose2d scoreTomahawkClosestReefPose() {
        var crPose = closestReefPose();
        var crPoseNativeRot = new Pose2d(crPose.getTranslation(),
                crPose.getRotation().minus(Constants.SCORING_SIDE_FROM_FRONT_ROT));
        var plusCoral = crPoseNativeRot
                .transformBy(new Transform2d(Units.inchesToMeters(Constants.BACKUP_DIST_IN), 0, Rotation2d.kZero));
        return new Pose2d(plusCoral.getTranslation(), crPose.getRotation());
    }

    public int closestReefTag() {
        return closestReefTag.ID;
    }

    public String closestReefName() {
        return closestReefName;
    }

    public Rotation2d closestReefL1Rotation() {
        return closestReefTag.pose.toPose2d().getRotation().plus(Constants.SCORING_SIDE_FROM_FRONT_ROT.unaryMinus());
    }

    public Command pathfindToPose(Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, new PathConstraints(2, 2, Math.PI, 2 * Math.PI));
    }

    public BooleanSupplier inPidTranslation() {
        return () -> inPidTranslate;
    }

    public BooleanSupplier readyForArm() {
        return () -> getPose().getTranslation()
                .getDistance(closestReef.getTranslation()) < Constants.ARM_READY_AUTO_SCORE_RADIUS;
    }

    public BooleanSupplier readyForArm(Armistice.ArmisticePositions pos) {
        return () -> getPose().getTranslation()
                .getDistance(closestReef.getTranslation()) < (pos.unSCPose().getUnPipe() == Armistice.ArmisticePositions.Cora_L2 ? 0.8 : Constants.ARM_READY_AUTO_SCORE_RADIUS);
    }

    @SuppressWarnings("removal")
    public Command translateToPositionWithPID(Pose2d pose) {
        DoubleSupplier theta = () -> new Pose2d(pose.getTranslation(), new Rotation2d())
                .relativeTo(new Pose2d(getPose().getTranslation(), new Rotation2d()))
                .getTranslation().getAngle().getRadians();
        DoubleSupplier driveYaw = () -> (getRotation().getRadians() + 2 * Math.PI) % (2 * Math.PI);
        return new PIDCommand(pidLineup,
                () -> pidErrorM = -new Pose2d(pose.getTranslation(), new Rotation2d())
                        .relativeTo(new Pose2d(getPose().getTranslation(), new Rotation2d())).getTranslation()
                        .getNorm(),
                0, (d) -> {
                    pidThetaR = theta.getAsDouble();
                    inPidTranslate = true;
                    if (pidLineup.atSetpoint()) {
                        stop();
                        return;
                    }
                    runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                            MathUtils.clamp(d * Math.cos(theta.getAsDouble()), -PID_TRANSLATION_SPEED_MPS,
                                    PID_TRANSLATION_SPEED_MPS),
                            MathUtils.clamp(d * Math.sin(theta.getAsDouble()), -PID_TRANSLATION_SPEED_MPS,
                                    PID_TRANSLATION_SPEED_MPS),
                            MathUtils.clamp(
                                    angleController.calculate(driveYaw.getAsDouble(),
                                            pose.getRotation().getRadians()),
                                    -PID_ROTATION_RAD_PER_SEC, PID_ROTATION_RAD_PER_SEC)),
                            getRotation()));
                }, this).finallyDo(i -> {
                    inPidTranslate = false;
                    pidErrorM = 0;
                    pidThetaR = 0;
                    pidLineup.reset();
                    angleController.reset();
                    stop();
                });
    }

    public BooleanSupplier translatePidInPosition() {
        return () -> pidLineup.atSetpoint() && angleController.atSetpoint();
    }

    public BooleanSupplier translatePidInPositionAndStop() {
        return () -> pidLineup.atSetpoint() && angleController.atSetpoint();
    }

    public Command waitForDrivetrainDistance(double posError) {
        return Commands.waitUntil(() -> inPidTranslate && Math.abs(pidErrorM * Math.cos(pidThetaR)) <= posError
                && Math.abs(pidErrorM * Math.sin(pidThetaR)) <= Math.min(posError, Units.inchesToMeters(0.5))
                && Math.abs(getChassisSpeeds().vxMetersPerSecond) <= 0.1
                && angleController.atSetpoint());
    }

    public BooleanSupplier hasPipeAtReef(Armistice armistice) {
        return () -> (inPidTranslate && (((!pidLineup.atSetpoint())
                && getChassisSpeeds().get2dVelocity() < 0.1
                && getPose().getTranslation().getDistance(pipe1ClosestReefPose().getTranslation()) < 0.1)
                && Arrays.stream(modules).allMatch(m -> m.getDriveCurrent() > 50)));
    }

    public BooleanSupplier translatePidInPositionJankier() {
        return () -> pidLineup.getError() <= 0.05 && angleController.atSetpoint();
    }

    public DoubleSupplier get2dFilteredX() {
        return () -> filteredX;
    }

    public DoubleSupplier get2dFilteredY() {
        return () -> filteredY;
    }

    public DoubleSupplier get2dFilteredRot() {
        return () -> filteredRot;
    }

    public Command llLineup2d(Limelight sourceLimelight, int tagID, DoubleSupplier targetTx, DoubleSupplier targetTy,
            DoubleSupplier targetRotDegrees) {
        return runOnce(() -> {
            isFinished2dAlign = false;
            limelightLineupSource2d = sourceLimelight;
            ll2dLineupTagID = tagID;
        }).andThen(run(() -> {
            boolean tv = limelightLineupSource2d.getTV();
            if (!tv) {
                for (var ll : VisionUtil.registeredLimelights()) {
                    if (ll.getTV() && ll.getTagID() == tagID)
                        limelightLineupSource2d = ll;
                }
            }
            double tx = get2dFilteredX().getAsDouble();
            double ty = Math.sqrt(get2dFilteredY().getAsDouble());
            double tagRot = get2dFilteredRot().getAsDouble();
            double xOutput, yOutput, rotOutput;
            double xErr = targetTx.getAsDouble() - tx;
            double yErr = targetTy.getAsDouble() - ty;
            double rotErr = targetRotDegrees.getAsDouble() - tagRot;

            isFinished2dAlign = Math.abs(xErr) <= .5 && Math.abs(yErr) <= .5 && Math.abs(rotErr) < 1;

            if (tv) {
                xOutput = Math.abs(xErr) > 0.5 ? -xPid.calculate(tx, targetTx.getAsDouble()) : 0;
                yOutput = Math.abs(yErr) > 0.5 ? yPid.calculate(ty, targetTy.getAsDouble()) : 0;
                rotOutput = Math.abs(rotErr) > 1 ? rotPid.calculate(tagRot, targetRotDegrees.getAsDouble()) : 0;
                var xTmp = xOutput;
                xOutput = xOutput * Math.cos(Units.degreesToRadians(tagRot))
                        + yOutput * Math.sin(Units.degreesToRadians(tagRot));
                yOutput = xTmp * -Math.sin(Units.degreesToRadians(tagRot))
                        + yOutput * Math.cos(Units.degreesToRadians(tagRot));
            } else {
                xOutput = yOutput = rotOutput = 0;
            }

            runVelocity(new ChassisSpeeds(yOutput, xOutput, rotOutput));
        }));
    }

    public BooleanSupplier isFinishedAligning2d() {
        return () -> isFinished2dAlign;
    }

    public boolean getReefTargetIsRight() {
        return reefTargetIsRight;
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the
     * modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /**
     * Returns the average velocity of the modules in rotations/sec (Phoenix native
     * units).
     */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
        VisionUtil.requestingSeed = true;
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
                new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
                new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }
}