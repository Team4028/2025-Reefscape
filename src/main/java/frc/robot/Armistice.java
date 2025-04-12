package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.JSONObject;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIOCanEncoderTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.util.LoggedTunables.LoggedTunableNumber;
import frc.robot.util.MathUtils;
import frc.robot.util.SudoSubsystem;
import lombok.NonNull;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod(frc.robot.util.RobotSim.class)
public class Armistice extends SudoSubsystem {

    public static final record SimData(double elevatorPositionMeters, double armPositionRadians) {
    }

    @AutoLogOutput
    private double elevatorTargetInches = ArmisticePositions.STOW.getElevatorPositionInches(0);
    @AutoLogOutput
    private double armTargetRad = ArmisticePositions.STOW.getArmPositionRad(0);
    @AutoLogOutput
    private ArmisticePositions targetArmisticePosition = ArmisticePositions.STOW;

    @AutoLogOutput
    private double globalElevatorOffsetInches = 0;

    private double globalArmOffsetRad = 0;

    @AutoLogOutput
    private boolean coralMode = true;

    @AutoLogOutput
    private final BooleanSupplier algaeSafety;

    @AutoLogOutput
    private boolean elevatorWaiting = true;

    private static final double[] ARM_SAFE_RANGE = new double[] { 14, 50 };
    private static final double[] ARM_SAFE_RANGE_ALG = new double[] {
            15 + Units.metersToInches(Constants.ALGAE_RADIUS_M) * 2, 50 };
    private static boolean useSafety = true;

    @AutoLogOutput
    private LoggedTunableNumber armCharVoltage = new LoggedTunableNumber("Arm Char Voltage", 0);

    @AutoLogOutput
    private ArmisticePositions futureArmisticePositions = ArmisticePositions.Cora_L4;

    @AutoLogOutput
    private ArmisticePositions futureAutoAlgaePosition = ArmisticePositions.A2_lgae;

    @AutoLogOutput
    private boolean magicAlgaeOn = true;

    @AutoLogOutput
    private int coralAquireOffset = 0;

    private static final int SAFETY_TOLERANCE = 4;

    @AutoLogOutput
    private boolean coralReefAcquireOffset = false;
    private JSONObject heatmapOffset = null;

    private Map<ArmisticePositions, ArmisticePositions> positionsMap = Map.of(
            ArmisticePositions.Cora_L1, ArmisticePositions.GROND,
            ArmisticePositions.Cora_L2, ArmisticePositions.A2_lgae,
            ArmisticePositions.Cora_L3, ArmisticePositions.A3_lgae,
            ArmisticePositions.Cora_L4, ArmisticePositions.BARGE);

    private Map<String, ArmisticePositions> reefAlgaeMap = Map.of(
            "12oC", ArmisticePositions.A2_lgae,
            "2oC", ArmisticePositions.A3_lgae,
            "4oC", ArmisticePositions.A2_lgae,
            "6oC", ArmisticePositions.A3_lgae,
            "8oC", ArmisticePositions.A2_lgae,
            "10oC", ArmisticePositions.A3_lgae);

    public static final double GLOBAL_ARM_OFFSET = Units.degreesToRadians(-8);
    public static final double GLOBAL_ELEVATOR_OFFSET = 0;

    public static enum ArmisticePositions {
        STOW(5.8 - 4, 5),
        CLEAN(3.902 + Units.degreesToRadians(2), 4.017),
        SHANK(2.62 - 4 + 2 * Math.PI, 7),
        Cora_L1(0.378 - 4 + 2 * Math.PI, 0), // 3.78
        Cora_L1_PIPE(0, 0),
        Cora_L2(4.738 - 4, 0),
        Cora_L2_SC(4.2 - 4, 0),
        Cora_L2_PIPE(0.926 + Units.degreesToRadians(2), 0),
        Cora_L2_PIPE_SC(4.2 - 4, 0),
        Cora_L3(5.3 - 4, 8.011),
        Cora_L3_SC(4.4 - 4, 8.011),
        Cora_L3_PIPE(0.987 + Units.degreesToRadians(2), 18),
        Cora_L3_PIPE_SC(0, 18),
        Cora_L4(5.5 - Units.degreesToRadians(7) - 4, 31.008),
        Cora_L4_SC(4.652 - 4, 31.008),
        Cora_L4_PIPE(1.11 + Units.degreesToRadians(2), 37.008),
        Cora_L4_PIPE_SC(4.29 - 4, 37.008),
        A2_lgae(0.287 + Units.degreesToRadians(3), 8),
        A3_lgae(4.247 - 4, 26),
        GROND(3.602 - 4, 0),
        PROC(-0.041, 0),
        BARGE(2.373 + Units.degreesToRadians(2), 44),
        CLIMB(3.902 + Units.degreesToRadians(2), 4.017), // negative version of this
        CLIMB_2(2 * Math.PI, 8.125),
        BARGE_ALT(1.515 - 0.52359878, 55);

        public final double armPositionRad;
        public final double elevatorPositionInches;
        public double armOffsetRad;
        public double elevatorOffsetInches;

        private ArmisticePositions(double armPositionRad, double elevatorPositionInches) {
            this.armPositionRad = armPositionRad + GLOBAL_ARM_OFFSET;
            this.elevatorPositionInches = elevatorPositionInches + GLOBAL_ELEVATOR_OFFSET;
            armOffsetRad = 0;
            elevatorOffsetInches = 0;
        }

        public double getElevatorPositionInches(double elevatorGlobalOffset) {
            return elevatorPositionInches + elevatorOffsetInches + elevatorGlobalOffset;
        }

        public double getArmPositionRad(double armGlobalOffset) {
            return armPositionRad + armOffsetRad + armGlobalOffset;
        }

        public boolean isSC() {
            return name().contains("SC");
        }

        public boolean isPipe() {
            return name().contains("PIPE");
        }

        public boolean isCoralScore() {
            return name().contains("Cora");
        }

        public boolean isAlgae() {
            return name().contains("lgae") || this == BARGE;
        }

        public boolean isClimb() {
            return name().contains("CLIMB");
        }

        public boolean isAcquire() {
            return this == CLEAN;
        }

        public boolean isAquireAlgae() {
            return this != BARGE && isAlgae();
        }

        public ArmisticePositions getUnPipe() {
            switch (this) {
                case Cora_L1_PIPE:
                    return Cora_L1;
                case Cora_L2_PIPE:
                    return Cora_L2;
                case Cora_L3_PIPE:
                    return Cora_L3;
                case Cora_L4_PIPE:
                    return Cora_L4;
                case Cora_L2_PIPE_SC:
                    return Cora_L2_SC;
                case Cora_L3_PIPE_SC:
                    return Cora_L3_SC;
                case Cora_L4_PIPE_SC:
                    return Cora_L4_SC;
                default:
                    return this;
            }
        }

        public ArmisticePositions toPipe() {
            switch (this) {
                case Cora_L1:
                    return Cora_L1_PIPE;
                case Cora_L2:
                    return Cora_L2_PIPE;
                case Cora_L3:
                    return Cora_L3_PIPE;
                case Cora_L4:
                    return Cora_L4_PIPE;
                case Cora_L2_SC:
                    return Cora_L2_PIPE_SC;
                case Cora_L3_SC:
                    return Cora_L3_PIPE_SC;
                case Cora_L4_SC:
                    return Cora_L4_PIPE_SC;
                default:
                    return this;
            }
        }

        public ArmisticePositions getSCPose() {
            switch (this) {
                case Cora_L2:
                    return Cora_L2_SC;
                case Cora_L2_PIPE:
                    return Cora_L2_PIPE_SC;
                case Cora_L3:
                    return Cora_L3_SC;
                case Cora_L3_PIPE:
                    return Cora_L3_PIPE_SC;
                case Cora_L4:
                    return Cora_L4_SC;
                case Cora_L4_PIPE:
                    return Cora_L4_PIPE_SC;
                default:
                    return this;
            }
        }

        public ArmisticePositions unSCPose() {
            switch (this) {
                case Cora_L2_SC:
                    return Cora_L2;
                case Cora_L2_PIPE_SC:
                    return Cora_L2_PIPE;
                case Cora_L3_SC:
                    return Cora_L3;
                case Cora_L3_PIPE_SC:
                    return Cora_L3_PIPE;
                case Cora_L4_SC:
                    return Cora_L4;
                case Cora_L4_PIPE_SC:
                    return Cora_L4_PIPE;
                default:
                    return this;
            }
        }
    }

    public Armistice(BooleanSupplier hasAlgae) {
        File offsetInput = new File(Filesystem.getDeployDirectory(), "HeatmapReefOffsets.json");
        new Trigger(hasAlgae).onTrue(Commands.runOnce(() -> disarm.setArmAccel(ArmConstants.ARM_ACCEL_W_ALGAE)))
                .onFalse(Commands.runOnce(() -> disarm.setArmAccel(ArmConstants.pidConfig.maxAccel())));
        this.algaeSafety = hasAlgae;
        if (offsetInput.isFile()) {
            try {
                String content = new String(Files.readAllBytes(Paths.get(offsetInput.getAbsolutePath())));
                heatmapOffset = new JSONObject(content);
            } catch (IOException ignored) {
            }
        }
    }

    private final Elevator summit = new ElevatorIOTalonFX().simSwitch();
    private final Arm disarm = new ArmIOCanEncoderTalonFX().simSwitch();

    public void getCanMagPosition() {
        disarm.getCanMagPosition();
    }

    public double[] getSafetyAlgaeCompensation() {
        // return algaeSafety.getAsBoolean() ? ARM_SAFE_RANGE_ALG : ARM_SAFE_RANGE;
        return ARM_SAFE_RANGE;
    }

    public void runArmVbus(double vbus) {
        disarm.runMotor(vbus);
    }

    public void runElevatorVbus(double vbus) {
        summit.runMotors(vbus);
    }

    public void disableArm() {
        disarm.runMotor(0);
    }

    public Command sysIDCommandElevator(BooleanSupplier dynamic, Supplier<Direction> direction) {
        return summit.sysIDTest(dynamic.getAsBoolean(), direction.get());
    }

    public Command runArmVoltageForChar() {
        return Commands.runOnce(() -> disarm.runMotor(Volts.of(armCharVoltage.get())), disarm);
    }

    public Command stopArm() {
        return Commands.runOnce(() -> disarm.runMotor(0), disarm);
    }

    public void setSafety(boolean isSafe) {
        useSafety = isSafe;
    }

    public boolean getSafety() {
        return useSafety;
    }

    public Command runToPositionCommand(ArmisticePositions position) {
        return Commands.runOnce(() -> {
            targetArmisticePosition = position;
            elevatorTargetInches = position.getElevatorPositionInches(globalElevatorOffsetInches);
            armTargetRad = position.getArmPositionRad(globalArmOffsetRad);
        }, summit, disarm).alongWith(Commands.waitUntil(armAndElevatorAtTarget()));
    }

    public Command runToPositionCommand(ArmisticePositions position, String closestReefName, boolean isRight) {
        return Commands.runOnce(() -> {
            targetArmisticePosition = position;
            double[] jsonOffsets = getJSONOffsets(closestReefName, position, isRight);
            elevatorTargetInches = position.getElevatorPositionInches(globalElevatorOffsetInches) + jsonOffsets[0];
            armTargetRad = position.getArmPositionRad(globalArmOffsetRad) + jsonOffsets[1];
        }, summit, disarm).alongWith(Commands.waitUntil(armAndElevatorAtTarget()));
    }

    public Command runToPositionNoWait(ArmisticePositions position) {
        return Commands.runOnce(() -> {
            targetArmisticePosition = position;
            elevatorTargetInches = position.getElevatorPositionInches(globalElevatorOffsetInches);
            armTargetRad = position.getArmPositionRad(globalArmOffsetRad);
        }, summit, disarm);
    }

    public Command runToPositionNoWait(ArmisticePositions position, String closestReefName, boolean isRight) {
        return Commands.runOnce(() -> {
            targetArmisticePosition = position;
            double[] jsonOffsets = getJSONOffsets(closestReefName, position, isRight);
            elevatorTargetInches = position.getElevatorPositionInches(globalElevatorOffsetInches) + jsonOffsets[0];
            armTargetRad = position.getArmPositionRad(globalArmOffsetRad) + jsonOffsets[1];
        }, summit, disarm);
    }

    public Command nudgeCommand(double elevatorInches, double armRad) {
        return Commands.runOnce(() -> {
            elevatorTargetInches += elevatorInches;
            armTargetRad += armRad;
        }, summit, disarm);
    }

    public Command nudgeCommandPermanant(double elevatorInches, double armRad) {
        return Commands.runOnce(() -> {
            elevatorTargetInches += elevatorInches;
            armTargetRad += armRad;
            targetArmisticePosition.armOffsetRad += armRad;
            targetArmisticePosition.elevatorOffsetInches += elevatorInches;
        }, summit, disarm);
    }

    public Command nudgeCommandGlobalPermanant(double elevatorInches, double armRad) {
        return Commands.runOnce(() -> {
            elevatorTargetInches += elevatorInches;
            armTargetRad += armRad;
            globalElevatorOffsetInches += elevatorInches;
            globalArmOffsetRad += armRad;
        });
    }

    public Command toggleCoralReefOffset() {
        return Commands.runOnce(() -> coralReefAcquireOffset = !coralReefAcquireOffset).ignoringDisable(true);
    }

    public Command setCoralReefOffset(boolean isPipe) {
        return Commands.runOnce(() -> coralReefAcquireOffset = isPipe).ignoringDisable(true);
    }

    public boolean getCoralReefOffset() {
        return coralReefAcquireOffset;
    }

    public Command runToFutureArmisticePositionCommand() {
        return Commands.defer(() -> runToPositionCommand(futureArmisticePositions), Set.of(disarm, summit));
    }

    public Command runToFutureArmisticePositionCommand(Supplier<String> closestReefName, BooleanSupplier isRight) {
        return Commands.defer(
                () -> runToPositionCommand(futureArmisticePositions, closestReefName.get(), isRight.getAsBoolean()),
                Set.of(disarm, summit));
    }

    public Command incFutureArmisticePosition() {
        return Commands.runOnce(() -> futureArmisticePositions = switch (futureArmisticePositions) {
            case Cora_L1 -> coralReefAcquireOffset ? ArmisticePositions.Cora_L2_PIPE : ArmisticePositions.Cora_L2;
            case Cora_L2 -> coralReefAcquireOffset ? ArmisticePositions.Cora_L3_PIPE : ArmisticePositions.Cora_L3;
            case Cora_L2_PIPE -> coralReefAcquireOffset ? ArmisticePositions.Cora_L3_PIPE : ArmisticePositions.Cora_L3;
            case Cora_L3 -> coralReefAcquireOffset ? ArmisticePositions.Cora_L4_PIPE : ArmisticePositions.Cora_L4;
            case Cora_L3_PIPE -> coralReefAcquireOffset ? ArmisticePositions.Cora_L4_PIPE : ArmisticePositions.Cora_L4;
            case Cora_L4 -> ArmisticePositions.Cora_L1;
            case Cora_L4_PIPE -> ArmisticePositions.Cora_L1;
            case GROND -> ArmisticePositions.A2_lgae;
            case A2_lgae -> ArmisticePositions.A3_lgae;
            case A3_lgae -> ArmisticePositions.BARGE;
            case BARGE -> ArmisticePositions.GROND;
            default -> ArmisticePositions.STOW;
        }).ignoringDisable(true);
    }

    public Command decFutureArmisticePosition() {
        return Commands.runOnce(() -> futureArmisticePositions = switch (futureArmisticePositions) {
            case Cora_L1 -> ArmisticePositions.Cora_L4;
            case Cora_L2, Cora_L2_PIPE -> ArmisticePositions.Cora_L1;
            case Cora_L3 -> coralReefAcquireOffset ? ArmisticePositions.Cora_L2_PIPE : ArmisticePositions.Cora_L2;
            case Cora_L3_PIPE ->
                coralReefAcquireOffset ? ArmisticePositions.Cora_L2_PIPE : ArmisticePositions.Cora_L2;
            case Cora_L4 -> coralReefAcquireOffset ? ArmisticePositions.Cora_L3_PIPE : ArmisticePositions.Cora_L3;
            case Cora_L4_PIPE -> coralReefAcquireOffset ? ArmisticePositions.Cora_L3_PIPE : ArmisticePositions.Cora_L3;
            case GROND -> ArmisticePositions.BARGE;
            case A2_lgae -> ArmisticePositions.GROND;
            case A3_lgae -> ArmisticePositions.A2_lgae;
            case BARGE -> ArmisticePositions.A3_lgae;
            default -> ArmisticePositions.STOW;
        }).ignoringDisable(true);
    }

    public Command setFutureArmisticePosition(ArmisticePositions position) {
        return Commands.runOnce(() -> futureArmisticePositions = position).ignoringDisable(true);
    }

    public Command toggleAutoAlgae() {
        return Commands.runOnce(() -> magicAlgaeOn = !magicAlgaeOn).ignoringDisable(true);
    }

    public boolean getMagicAlgaeOn() {
        return magicAlgaeOn;
    }

    public Command resetNudges() {
        return Commands.runOnce(() -> Arrays.asList(ArmisticePositions.values()).forEach(
                p -> globalElevatorOffsetInches = globalArmOffsetRad = p.armOffsetRad = p.elevatorOffsetInches = 0))
                .ignoringDisable(true);
    }

    @AutoLogOutput
    public BooleanSupplier armAndElevatorAtTarget() {
        return () -> disarm.atTargetPosition().getAsBoolean() && summit.atTargetPosition().getAsBoolean();
    }

    public ArmisticePositions getTargetPosition() {
        return targetArmisticePosition;
    }

    public SimData getSimData() {
        return new SimData(summit.getSimPos(), disarm.getSimAngle());
    }

    public double getElevatorPosition() {
        return summit.getCurrentPosition();
    }

    public SubsystemBase[] getSubsystems() {
        return new SubsystemBase[] { disarm, summit };
    }

    public ArmisticePositions getFutureArmisticePositions() {
        return futureArmisticePositions;
    }

    public ArmisticePositions getAutoAlgaePosition() {
        return futureAutoAlgaePosition;
    }

    public Arm getArm() {
        return disarm;
    }

    public Elevator getElevator() {
        return summit;
    }

    public boolean elevatorIsSafe() {
        var safeR = getSafetyAlgaeCompensation();
        return MathUtils.inRangeWithTolerance(summit.getCurrentPosition(), safeR[0], safeR[1], SAFETY_TOLERANCE)
                && MathUtils.inRangeWithTolerance(elevatorTargetInches, safeR[0], safeR[1], SAFETY_TOLERANCE);
    }

    public Command stageArm(ArmisticePositions position) {
        return Commands.runOnce(() -> armTargetRad = position.getArmPositionRad(globalArmOffsetRad))
                .andThen(Commands.waitUntil(this::disarmAtRealTarget));
    }

    // holding arm while compensatively moving elevator updates arm internal target
    // pos; therefore this
    public boolean disarmAtRealTarget() {
        return Math.abs(armTargetRad - disarm.getCurrentPosition()) <= ArmConstants.PID_TOLERANCE;
    }

    public boolean disarmAtSafeDistance() {
        return Math.abs(armTargetRad - disarm.getCurrentPosition()) <= ArmConstants.SAFE_DISTANCE;
    }

    public Command waitUntilThingsInTolerance(double elevatorTol, double armTol) {
        return Commands.waitUntil(() -> Math.abs(elevatorTargetInches - summit.getCurrentPosition()) <= elevatorTol
                && Math.abs(armTargetRad - disarm.getCurrentPosition()) <= armTol);
    }

    public Command runElevator(ArmisticePositions position) {
        return Commands
                .runOnce(() -> elevatorTargetInches = position.getElevatorPositionInches(globalElevatorOffsetInches))
                .andThen(Commands.waitUntil(summit.atTargetPosition()));
    }

    public boolean magicIsSnap() {
        return futureArmisticePositions == ArmisticePositions.Cora_L1;
    }

    public boolean getCoralMode() {
        return coralMode;
    }

    public Command toggleCoralMode() {
        return Commands.runOnce(() -> coralMode = !coralMode).ignoringDisable(true);
    }

    public void updateAutoAlgaePos(int reefTag) {
        futureAutoAlgaePosition = magicAlgaeOn ? reefAlgaeMap.get(Constants.reefTagNames.get(reefTag))
                : ArmisticePositions.BARGE;
    }

    /** Elevator Offset first (in), then Arm Offset (rad) */
    public double[] getJSONOffsets(@NonNull String closestReefName, @NonNull ArmisticePositions targetPosition,
            boolean isRight) {
        targetPosition = targetPosition.unSCPose();
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        if (heatmapOffset == null)
            return new double[] { 0, 0 };
        else if (targetPosition == ArmisticePositions.BARGE) {
            var armEJson = heatmapOffset.getJSONObject(isBlue ? "blue" : "red").getJSONObject("BARGE");
            return new double[] {
                    armEJson.getDouble("elevator"),
                    armEJson.getDouble("arm")
            };
        } else if (targetPosition.isCoralScore()) {
            var armEJson = heatmapOffset.getJSONObject(isBlue ? "blue" : "red").getJSONObject(closestReefName)
                    .getJSONObject(isRight ? "right" : "left")
                    .getJSONObject(targetPosition.name());
            return new double[] {
                    armEJson.getDouble("elevator"),
                    armEJson.getDouble("arm")
            };
        } else if (targetPosition.isAquireAlgae()) {
            var armEJson = heatmapOffset.getJSONObject(isBlue ? "blue" : "red").getJSONObject(closestReefName)
                    .getJSONObject(targetPosition.name());
            return new double[] {
                    armEJson.getDouble("elevator"),
                    armEJson.getDouble("arm")
            };
        } else if (targetPosition.isAcquire()) {
            var armEJson = heatmapOffset.getJSONObject(isBlue ? "blue" : "red").getJSONObject(targetPosition.name());
            return new double[] {
                    armEJson.getDouble("elevator"),
                    armEJson.getDouble("arm"),
            };
        } else {
            return new double[] { 0, 0 };
        }
    }

    @Override
    public void periodic() {
        if (Constants.USE_ARMISTICE_PID) {
            if (useSafety) {
                var safeR = getSafetyAlgaeCompensation();
                elevatorWaiting = !disarmAtSafeDistance() && !elevatorIsSafe();
                if (elevatorIsSafe() || (elevatorWaiting && summit.withinRange(SAFETY_TOLERANCE))
                        || disarmAtSafeDistance()) {
                    disarm.runToPosition(armTargetRad);
                    summit.runToPosition(
                            elevatorWaiting
                                    ? MathUtils.clamp(elevatorTargetInches, safeR[0], safeR[1])
                                    : elevatorTargetInches);
                } else {
                    disarm.runToPosition(
                            MathUtils.inRangeWithTolerance(summit.getCurrentPosition(), safeR[0], safeR[1],
                                    SAFETY_TOLERANCE)
                                            ? armTargetRad
                                            : disarm.getCurrentPosition());
                    summit.runToPosition(MathUtils.clamp(elevatorTargetInches, safeR[0], safeR[1]));
                }
            } else {
                disarm.runToPosition(armTargetRad);
                summit.runToPosition(elevatorTargetInches);
            }
        }
    }

    @Override
    public void robotPeriodic() {
        Logger.recordOutput("Armistice/ElevatorOffsets/FutureArmisticePositionOffset",
                futureArmisticePositions.elevatorOffsetInches);
        Logger.recordOutput("Armistice/ElevatorOffsets/AutoAlgaeCounterOffset",
                futureAutoAlgaePosition.elevatorOffsetInches);
        Logger.recordOutput("Armistice/ElevatorOffsets/StowPositionOffset",
                ArmisticePositions.STOW.elevatorOffsetInches);
        Logger.recordOutput("Armistice/ArmOffsets/FutureArmisticePositionOffset",
                MathUtils.roundToPlace(Units.radiansToDegrees(futureArmisticePositions.armOffsetRad), 3));
        Logger.recordOutput("Armistice/ArmOffsets/AutoAlgaeCounterOffset",
                MathUtils.roundToPlace(Units.radiansToDegrees(futureAutoAlgaePosition.armOffsetRad), 3));
        Logger.recordOutput("Armistice/ArmOffsets/StowPositionOffset",
                MathUtils.roundToPlace(Units.radiansToDegrees(ArmisticePositions.STOW.armOffsetRad), 3));
        Logger.recordOutput("Armistice/ElevatorTargetInchesFriendly", MathUtils.roundToPlace(elevatorTargetInches, 3));
        Logger.recordOutput("Armistice/ArmTargetRadFriendly", MathUtils.roundToPlace(armTargetRad, 3));
        Logger.recordOutput("Armistice/ArmGlobalOffsetDegFriendly",
                MathUtils.roundToPlace(Units.radiansToDegrees(globalArmOffsetRad), 3));

        if ((coralMode && !positionsMap.containsKey(futureArmisticePositions.getUnPipe()))
                || (!coralMode && !positionsMap.containsValue(futureArmisticePositions.getUnPipe()))) {
            for (var e : positionsMap.entrySet()) {
                if (futureArmisticePositions.getUnPipe() == e.getValue()) {
                    futureArmisticePositions = e.getKey();
                    break;
                } else if (futureArmisticePositions.getUnPipe() == e.getKey()) {
                    futureArmisticePositions = e.getValue();
                    break;
                }
            }
        }

        if (coralReefAcquireOffset && (futureArmisticePositions == ArmisticePositions.Cora_L2
                || futureArmisticePositions == ArmisticePositions.Cora_L3
                || futureArmisticePositions == ArmisticePositions.Cora_L4)) {
            futureArmisticePositions = futureArmisticePositions == ArmisticePositions.Cora_L2
                    ? ArmisticePositions.Cora_L2_PIPE
                    : futureArmisticePositions == ArmisticePositions.Cora_L3
                            ? ArmisticePositions.Cora_L3_PIPE
                            : ArmisticePositions.Cora_L4_PIPE;
        } else if (!coralReefAcquireOffset && (futureArmisticePositions == ArmisticePositions.Cora_L2_PIPE
                || futureArmisticePositions == ArmisticePositions.Cora_L3_PIPE
                || futureArmisticePositions == ArmisticePositions.Cora_L4_PIPE)) {
            futureArmisticePositions = futureArmisticePositions == ArmisticePositions.Cora_L2_PIPE
                    ? ArmisticePositions.Cora_L2
                    : futureArmisticePositions == ArmisticePositions.Cora_L3_PIPE
                            ? ArmisticePositions.Cora_L3
                            : ArmisticePositions.Cora_L4;
        }
    }
}
