package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIOCanEncoderTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.util.MathUtils;
import frc.robot.util.SudoSubsystem;
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
    private boolean elevatorWaiting = true;

    private static final double[] ARM_SAFE_RANGE = new double[] { 5, 35 };
    private static final boolean USE_SAFETY = false;

    @AutoLogOutput
    private double armCharVoltage = 0;

    @AutoLogOutput
    private ArmisticePositions futureArmisticePositions = ArmisticePositions.Cora_L2;

    @AutoLogOutput
    private ArmisticePositions futureAutoAlgaePosition = ArmisticePositions.A2_lgae;

    @AutoLogOutput
    private boolean magicAlgaeOn = true;

    @AutoLogOutput
    private ArmisticePositions futureAquirePosition = ArmisticePositions.CLEAN;

    @AutoLogOutput
    private int coralAquireOffset = 0;

    private Map<Integer, ArmisticePositions> aquireOffsetMap = Map.of(
            0, ArmisticePositions.CLEAN,
            1, ArmisticePositions.PIPE1,
            2, ArmisticePositions.PIPE2);

    private Map<ArmisticePositions, ArmisticePositions> positionsMap = Map.of(
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

    public static enum ArmisticePositions {
        STOW(4.097 - 0.52359878, 9),
        CLEAN(0.855 - 0.52359878, 8.1),
        PIPE1(0.94 - 0.52359878, 6.1),
        PIPE2(1, 5),
        Cora_L2(4.097 - 0.52359878, 9),
        Cora_L3(4.097 - 0.52359878, 24.54),
        Cora_L4(3.907 - 0.52359878 - 0.05236, 55.0),
        A2_lgae(5.624 - 0.52359878, 9.14),
        A3_lgae(5.624 - 0.52359878, 25.14),
        LOLI(2.68 - 0.52359878, 0.61),
        BARGE(6.14 - 0.52359878, 55),
        CLIMB(2.911 - 0.52359878, 0.5),
        BARGE_ALT(1.515 - 0.52359878, 55);

        public final double armPositionRad;
        public final double elevatorPositionInches;
        public double armOffsetRad;
        public double elevatorOffsetInches;

        public double getElevatorPositionInches(double elevatorGlobalOffset) {
            return elevatorPositionInches + elevatorOffsetInches + elevatorGlobalOffset;
        }

        public double getArmPositionRad(double armGlobalOffset) {
            return armPositionRad + armOffsetRad + armGlobalOffset;
        }

        private ArmisticePositions(double armPositionRad, double elevatorPositionInches) {
            this.armPositionRad = armPositionRad;
            this.elevatorPositionInches = elevatorPositionInches;
            armOffsetRad = 0;
            elevatorOffsetInches = 0;
        }
    }

    public Armistice() {
        NamedCommands.registerCommand("L4 Score", runToPositionCommand(ArmisticePositions.Cora_L4));
        NamedCommands.registerCommand("Stow", runToPositionCommand(ArmisticePositions.STOW));
        NamedCommands.registerCommand("Stow No Wait", runToPositionNoWait(ArmisticePositions.STOW));
        NamedCommands.registerCommand("Acquire Pos", runToPositionCommand(ArmisticePositions.CLEAN));
        NamedCommands.registerCommand("L3 Score", runToPositionCommand(ArmisticePositions.Cora_L3));
    }

    private final Elevator summit = new ElevatorIOTalonFX().simSwitch();
    private final Arm disarm = new ArmIOCanEncoderTalonFX().simSwitch();

    public void getCanMagPosition() {
        disarm.getCanMagPosition();
    }

    public void runArmVbus(double vbus) {
        disarm.runMotor(vbus);
    }

    public void runElevatorVbus(double vbus) {
        summit.runMotors(vbus);
    }

    /** Kills the arm(y) */
    public void orbitalStrike() {
        disarm.runMotor(0);
    }

    public Command sysIDCommandElevator(BooleanSupplier dynamic, Supplier<Direction> direction) {
        return summit.sysIDTest(dynamic.getAsBoolean(), direction.get());
    }

    public Command runArmVoltageForChar() {
        return Commands.runOnce(() -> disarm.runMotor(Volts.of(armCharVoltage)), disarm);
    }

    public Command stopArm() {
        return Commands.runOnce(() -> disarm.runMotor(0), disarm);
    }

    public Command deltaArmCharVolts(double dVolts) {
        return Commands.runOnce(() -> armCharVoltage += dVolts);
    }

    public Command runToPositionCommand(ArmisticePositions position) {
        return Commands.runOnce(() -> {
            targetArmisticePosition = position;
            elevatorTargetInches = position.getElevatorPositionInches(globalElevatorOffsetInches);
            armTargetRad = position.getArmPositionRad(globalArmOffsetRad);
        }, summit, disarm).alongWith(Commands.waitUntil(armAndElevatorAtTarget()));
    }

    public Command runToPositionNoWait(ArmisticePositions position) {
        return Commands.runOnce(() -> {
            targetArmisticePosition = position;
            elevatorTargetInches = position.getElevatorPositionInches(globalElevatorOffsetInches);
            armTargetRad = position.getArmPositionRad(globalArmOffsetRad);
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

    public Command runToFutureArmisticePositionCommand() {
        return Commands.defer(() -> runToPositionCommand(futureArmisticePositions), Set.of(disarm, summit));
    }

    public Command runToFutureAquirePositionCommand() {
        return Commands.defer(() -> runToPositionCommand(futureAquirePosition), Set.of(disarm, summit));
    }

    public Command incFutureArmisticePosition() {
        return Commands.runOnce(() -> futureArmisticePositions = switch (futureArmisticePositions) {
            case Cora_L2 -> ArmisticePositions.Cora_L3;
            case Cora_L3 -> ArmisticePositions.Cora_L4;
            case Cora_L4 -> ArmisticePositions.Cora_L2;
            case A2_lgae -> ArmisticePositions.A3_lgae;
            case A3_lgae -> ArmisticePositions.BARGE;
            case BARGE -> ArmisticePositions.A2_lgae;
            default -> ArmisticePositions.STOW;
        }).ignoringDisable(true);
    }

    public Command decFutureArmisticePosition() {
        return Commands.runOnce(() -> futureArmisticePositions = switch (futureArmisticePositions) {
            case Cora_L2 -> ArmisticePositions.Cora_L4;
            case Cora_L3 -> ArmisticePositions.Cora_L2;
            case Cora_L4 -> ArmisticePositions.Cora_L3;
            case A2_lgae -> ArmisticePositions.BARGE;
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

    public Command incFutureAquirePos() {
        return Commands.runOnce(() -> coralAquireOffset = (coralAquireOffset + 1) % aquireOffsetMap.size())
                .ignoringDisable(true);
    }

    public Command decFutureAquirePos() {
        return Commands.runOnce(() -> {
            coralAquireOffset = (coralAquireOffset - 1) % aquireOffsetMap.size();
            while (coralAquireOffset < 0)
                coralAquireOffset += aquireOffsetMap.size();
        })
                .ignoringDisable(true);
    }

    public Command resetNudges() {
        return Commands.runOnce(() -> Arrays.asList(ArmisticePositions.values()).forEach(
                p -> globalElevatorOffsetInches = globalArmOffsetRad = p.armOffsetRad = p.elevatorOffsetInches = 0))
                .ignoringDisable(true);
    }

    public BooleanSupplier armAndElevatorAtTarget() {
        return () -> disarm.atTargetPosition().getAsBoolean() && summit.atTargetPosition().getAsBoolean();
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

    public ArmisticePositions getFutureAquirePosition() {
        return futureAquirePosition;
    }

    public Arm getArm() {
        return disarm;
    }

    public Elevator getElevator() {
        return summit;
    }

    public boolean elevatorIsSafe() {
        return MathUtils.inRange(summit.getCurrentPosition(), ARM_SAFE_RANGE[0], ARM_SAFE_RANGE[1])
                && MathUtils.inRange(elevatorTargetInches, ARM_SAFE_RANGE[0], ARM_SAFE_RANGE[1]);
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

    public Command runElevator(ArmisticePositions position) {
        return Commands
                .runOnce(() -> elevatorTargetInches = position.getElevatorPositionInches(globalElevatorOffsetInches))
                .andThen(Commands.waitUntil(summit.atTargetPosition()));
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

    @Override
    public void periodic() {
        if (USE_SAFETY) {
            elevatorWaiting = !disarmAtRealTarget() && !elevatorIsSafe();
            if (elevatorIsSafe() || (elevatorWaiting && summit.atTargetPosition().getAsBoolean())
                    || disarmAtRealTarget()) {
                disarm.runToPosition(armTargetRad);
                summit.runToPosition(
                        elevatorWaiting ? MathUtils.clamp(elevatorTargetInches, ARM_SAFE_RANGE[0], ARM_SAFE_RANGE[1])
                                : elevatorTargetInches);
            } else {
                disarm.runToPosition(
                        MathUtils.inRange(summit.getCurrentPosition(), ARM_SAFE_RANGE[0], ARM_SAFE_RANGE[1])
                                ? armTargetRad
                                : disarm.getCurrentPosition());
                summit.runToPosition(MathUtils.clamp(elevatorTargetInches, ARM_SAFE_RANGE[0], ARM_SAFE_RANGE[1]));
            }
        } else {
            disarm.runToPosition(armTargetRad);
            summit.runToPosition(elevatorTargetInches);
        }

    }

    @Override
    public void robotPeriodic() {
        Logger.recordOutput("Armistice/ElevatorOffsets/FutureArmisticePositionOffset",
                futureArmisticePositions.elevatorOffsetInches);
        Logger.recordOutput("Armistice/ElevatorOffsets/AutoAlgaeCounterOffset",
                futureAutoAlgaePosition.elevatorOffsetInches);
        Logger.recordOutput("Armistice/ElevatorOffsets/FutureAquirePositionOffset",
                futureAquirePosition.elevatorOffsetInches);
        Logger.recordOutput("Armistice/ElevatorOffsets/StowPositionOffset",
                ArmisticePositions.STOW.elevatorOffsetInches);
        Logger.recordOutput("Armistice/ArmOffsets/FutureArmisticePositionOffset",
                MathUtils.roundToPlace(Units.radiansToDegrees(futureArmisticePositions.armOffsetRad), 3));
        Logger.recordOutput("Armistice/ArmOffsets/AutoAlgaeCounterOffset",
                MathUtils.roundToPlace(Units.radiansToDegrees(futureAutoAlgaePosition.armOffsetRad), 3));
        Logger.recordOutput("Armistice/ArmOffsets/FutureAquirePositionOffset",
                MathUtils.roundToPlace(Units.radiansToDegrees(futureAquirePosition.armOffsetRad), 3));
        Logger.recordOutput("Armistice/ArmOffsets/StowPositionOffset",
                MathUtils.roundToPlace(Units.radiansToDegrees(ArmisticePositions.STOW.armOffsetRad), 3));
        Logger.recordOutput("Armistice/ElevatorTargetInchesFriendly", MathUtils.roundToPlace(elevatorTargetInches, 3));
        Logger.recordOutput("Armistice/ArmTargetRadFriendly", MathUtils.roundToPlace(armTargetRad, 3));
        Logger.recordOutput("Armistice/ArmGlobalOffsetDegFriendly",
                MathUtils.roundToPlace(Units.radiansToDegrees(globalArmOffsetRad), 3));

        if ((coralMode && !positionsMap.containsKey(futureArmisticePositions))
                || (!coralMode && !positionsMap.containsValue(futureArmisticePositions))) {
            for (var e : positionsMap.entrySet()) {
                if (futureArmisticePositions == e.getValue()) {
                    futureArmisticePositions = e.getKey();
                    break;
                } else if (futureArmisticePositions == e.getKey()) {
                    futureArmisticePositions = e.getValue();
                    break;
                }
            }
        }

        futureAquirePosition = coralMode ? aquireOffsetMap.get(coralAquireOffset) : ArmisticePositions.LOLI;
    }
}
