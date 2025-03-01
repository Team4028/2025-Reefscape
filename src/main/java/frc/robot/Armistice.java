package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

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
import frc.robot.util.MathUtil;
import frc.robot.util.RobotSim;
import frc.robot.util.SudoSubsystem;

public class Armistice extends SudoSubsystem {

    public static final record SimData(double elevatorPositionMeters, double armPositionRadians) {
    }

    @AutoLogOutput
    private double elevatorTargetInches = ArmisticePositions.STOW.elevatorPositionInches;
    @AutoLogOutput
    private double armTargetRad = ArmisticePositions.STOW.armPositionRad;

    @AutoLogOutput
    private boolean elevatorWaiting = true;

    private static final double[] ARM_SAFE_RANGE = new double[] { 5, 35 };
    private static final boolean USE_SAFETY = true;

    @AutoLogOutput
    private double armCharVoltage = 0;
    private ArmisticePositions futureArmisticePositions = ArmisticePositions.L4;

    public static enum ArmisticePositions {
        STOW(0.43, 7.1),
        ACQUIRE(0.855, 8.1),
        ACQUIRE_BLOCKED(0.94, 6.1),
        L2(4.097, 9),
        L3(4.097, 24.54),
        L4(3.907, 55.0),
        ALGAE_AQUIRE_L2(5.624, 9.14),
        ALGAE_AQUIRE_L3(5.624, 25.14),
        LOLLIPOP_ACQUIRE(2.68, 0.61),
        BARGE_REAL(6.14, 55),
        BARGE_ALT(1.515, 55);

        public double armPositionRad;
        public double elevatorPositionInches;

        private ArmisticePositions(double armPositionRad, double elevatorPositionInches) {
            this.armPositionRad = armPositionRad;
            this.elevatorPositionInches = elevatorPositionInches;
        }
    }

    public Armistice() {
        NamedCommands.registerCommand("L4 Score", runToPositionCommand(ArmisticePositions.L4));
        NamedCommands.registerCommand("Stow", runToPositionCommand(ArmisticePositions.STOW));
        NamedCommands.registerCommand("Stow No Wait", runToPositionNoWait(ArmisticePositions.STOW));
        NamedCommands.registerCommand("Acquire Pos", runToPositionCommand(ArmisticePositions.ACQUIRE));
        NamedCommands.registerCommand("L3 Score", runToPositionCommand(ArmisticePositions.L3));
    }

    private final Elevator summit = RobotSim.elevatorSimSwitch(new ElevatorIOTalonFX());
    private final Arm disarm = RobotSim.armSimSwitch(new ArmIOCanEncoderTalonFX());

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

    public Command changeFutureArmisticePosition(int delta) {
        return Commands.runOnce(() -> futureArmisticePositions = switch (futureArmisticePositions) {
            case LOLLIPOP_ACQUIRE -> ArmisticePositions.L2;
            case L2 -> ArmisticePositions.L3;
            case L3 -> ArmisticePositions.L4;
            case L4 -> ArmisticePositions.BARGE_REAL;
            case BARGE_REAL -> ArmisticePositions.LOLLIPOP_ACQUIRE;
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

    public Arm getArm() {
        return disarm;
    }

    public Elevator getElevator() {
        return summit;
    }

    public boolean elevatorIsSafe() {
        return MathUtil.inRange(summit.getCurrentPosition(), ARM_SAFE_RANGE[0], ARM_SAFE_RANGE[1])
                && MathUtil.inRange(elevatorTargetInches, ARM_SAFE_RANGE[0], ARM_SAFE_RANGE[1]);
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

    @Override
    public void periodic() {
        if (USE_SAFETY) {
            elevatorWaiting = !disarmAtRealTarget() && !elevatorIsSafe();
            if (elevatorIsSafe() || (elevatorWaiting && summit.atTargetPosition().getAsBoolean())
                    || disarmAtRealTarget()) {
                disarm.runToPosition(armTargetRad);
                summit.runToPosition(
                        elevatorWaiting ? MathUtil.clamp(elevatorTargetInches, ARM_SAFE_RANGE[0], ARM_SAFE_RANGE[1])
                                : elevatorTargetInches);
            } else {
                disarm.runToPosition(MathUtil.inRange(summit.getCurrentPosition(), ARM_SAFE_RANGE[0], ARM_SAFE_RANGE[1])
                        ? armTargetRad
                        : disarm.getCurrentPosition());
                summit.runToPosition(MathUtil.clamp(elevatorTargetInches, ARM_SAFE_RANGE[0], ARM_SAFE_RANGE[1]));
            }
        } else {
            disarm.runToPosition(armTargetRad);
            summit.runToPosition(elevatorTargetInches);
        }
    }
}
