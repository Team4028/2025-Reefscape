package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

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
    private double elevatorTargetInches = ArmisticePositions.STOW.elevatorPositionInches;
    @AutoLogOutput
    private double armTargetRad = ArmisticePositions.STOW.armPositionRad;

    @AutoLogOutput
    private boolean coralMode = true;

    @AutoLogOutput
    private boolean elevatorWaiting = true;

    private static final double[] ARM_SAFE_RANGE = new double[] { 5, 35 };
    private static final boolean USE_SAFETY = true;

    @AutoLogOutput
    private double armCharVoltage = 0;

    private ArmisticePositions futureArmisticePositions = ArmisticePositions.Cora_L2;

    private ArmisticePositions futureAutoAlgaePosition = ArmisticePositions.A2_lgae;

    private Map<ArmisticePositions, ArmisticePositions> positionsMap = Map.of(
            ArmisticePositions.ACQUIRE, ArmisticePositions.LOLI,
            ArmisticePositions.Cora_L2, ArmisticePositions.A2_lgae,
            ArmisticePositions.Cora_L3, ArmisticePositions.A3_lgae,
            ArmisticePositions.Cora_L4, ArmisticePositions.BARGE);

    public static enum ArmisticePositions {
        STOW(4.097 - 0.52359878, 9),
        ACQUIRE(0.855 - 0.52359878, 8.1),
        ACQUIRE_BLOCKED(0.94 - 0.52359878, 6.1),
        Cora_L2(4.097 - 0.52359878, 9),
        Cora_L3(4.097 - 0.52359878, 24.54),
        Cora_L4(3.907 - 0.52359878, 55.0),
        A2_lgae(5.624 - 0.52359878, 9.14),
        A3_lgae(5.624 - 0.52359878, 25.14),
        LOLI(2.68 - 0.52359878, 0.61),
        BARGE(6.14 - 0.52359878, 55),
        CLIMB(2.911 - 0.52359878, 0.5),
        BARGE_ALT(1.515 - 0.52359878, 55);

        public double armPositionRad;
        public double elevatorPositionInches;

        private ArmisticePositions(double armPositionRad, double elevatorPositionInches) {
            this.armPositionRad = armPositionRad;
            this.elevatorPositionInches = elevatorPositionInches;
        }
    }

    public Armistice() {
        NamedCommands.registerCommand("L4 Score", runToPositionCommand(ArmisticePositions.Cora_L4));
        NamedCommands.registerCommand("Stow", runToPositionCommand(ArmisticePositions.STOW));
        NamedCommands.registerCommand("Stow No Wait", runToPositionNoWait(ArmisticePositions.STOW));
        NamedCommands.registerCommand("Acquire Pos", runToPositionCommand(ArmisticePositions.ACQUIRE));
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
            elevatorTargetInches = position.elevatorPositionInches;
            armTargetRad = position.armPositionRad;
        }, summit, disarm).alongWith(Commands.waitUntil(armAndElevatorAtTarget()));
    }

    public Command runToPositionNoWait(ArmisticePositions position) {
        return Commands.runOnce(() -> {
            elevatorTargetInches = position.elevatorPositionInches;
            armTargetRad = position.armPositionRad;
        }, summit, disarm);
    }

    public Command nudgeCommand(double elevatorInches, double armRad) {
        return Commands.runOnce(() -> {
            elevatorTargetInches += elevatorInches;
            armTargetRad += armRad;
        }, summit, disarm);
    }

    public Command runToFutureArmisticePositionCommand() {
        return Commands.defer(() -> runToPositionCommand(futureArmisticePositions), Set.of(disarm, summit));
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
        });
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
        });
    }

    public Command incAutoAlgaePos() {
        return Commands.runOnce(() -> futureAutoAlgaePosition = switch(futureAutoAlgaePosition) {
            case A2_lgae -> ArmisticePositions.A3_lgae;
            case A3_lgae -> ArmisticePositions.BARGE;
            case BARGE -> ArmisticePositions.A2_lgae;
            default -> ArmisticePositions.STOW;
        });
    }

    public Command decAutoAlgaePos() {
        return Commands.runOnce(() -> futureAutoAlgaePosition = switch(futureAutoAlgaePosition) {
            case A2_lgae -> ArmisticePositions.BARGE;
            case A3_lgae -> ArmisticePositions.A2_lgae;
            case BARGE -> ArmisticePositions.A3_lgae;
            default -> ArmisticePositions.STOW;
        });
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
        return Commands.runOnce(() -> armTargetRad = position.armPositionRad)
                .andThen(Commands.waitUntil(this::disarmAtRealTarget));
    }

    // holding arm while compensatively moving elevator updates arm internal target
    // pos; therefore this
    public boolean disarmAtRealTarget() {
        return Math.abs(armTargetRad - disarm.getCurrentPosition()) <= ArmConstants.PID_TOLERANCE;
    }

    public Command runElevator(ArmisticePositions position) {
        return Commands.runOnce(() -> elevatorTargetInches = position.elevatorPositionInches)
                .andThen(Commands.waitUntil(summit.atTargetPosition()));
    }

    public boolean getCoralMode() {
        return coralMode;
    }

    public void setCoralMode(boolean coralMode) {
        this.coralMode = coralMode;
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
                disarm.runToPosition(MathUtils.inRange(summit.getCurrentPosition(), ARM_SAFE_RANGE[0], ARM_SAFE_RANGE[1])
                        ? armTargetRad
                        : disarm.getCurrentPosition());
                summit.runToPosition(MathUtils.clamp(elevatorTargetInches, ARM_SAFE_RANGE[0], ARM_SAFE_RANGE[1]));
            }
        } else {
            disarm.runToPosition(armTargetRad);
            summit.runToPosition(elevatorTargetInches);
        }

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

        Logger.recordOutput("Armistice/FutureArmisticePosition", "A: " + futureArmisticePositions.name());
        Logger.recordOutput("Armistice/AutoAlgaeCounter", "B: " + futureAutoAlgaePosition.name());
    }
}
