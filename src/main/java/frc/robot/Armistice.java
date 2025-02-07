package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.arm.ArmConstants.ArmSafetyData;
import frc.robot.subsystems.elevator.*;
import frc.robot.util.MathUtil;
import frc.robot.util.RobotSim;
import frc.robot.util.SudoSubsystem;

public class Armistice extends SudoSubsystem {

    public static final record SimData(double elevatorPositionMeters, double armPositionRadians) {
    }

    @AutoLogOutput
    private boolean isInDanger = true;

    @AutoLogOutput
    private double elevatorTargetInches = ArmisticePositions.STOW.elevatorPositionInches;
    @AutoLogOutput
    private double armTargetRad = Units.degreesToRadians(ArmisticePositions.STOW.armPositionDeg);

    public static enum ArmisticePositions {
        STOW(180, 7),
        ACQUIRE(235, 15),
        L2(55, 37),
        L3(55, 51),
        L4(125, 57);

        public double armPositionDeg;
        public double elevatorPositionInches;

        private ArmisticePositions(double armPositionDeg, double elevatorPositionInches) {
            this.armPositionDeg = armPositionDeg;
            this.elevatorPositionInches = elevatorPositionInches;
        }
    }

    public Armistice() {
        NamedCommands.registerCommand("L4 Score", runToPositionCommand(() -> ArmisticePositions.L4));
        NamedCommands.registerCommand("Stow", runToPositionCommand(() -> ArmisticePositions.STOW));
        NamedCommands.registerCommand("L3 Score", runToPositionCommand(() -> ArmisticePositions.L3));
        NamedCommands.registerCommand("Stow Arm",
                Commands.runOnce(
                        () -> disarm.runToPosition(Units.degreesToRadians(ArmisticePositions.STOW.armPositionDeg)),
                        disarm).alongWith(Commands.waitUntil(armAndElevatorAtTarget())));
    }

    private final Elevator summit = new Elevator(RobotSim.elevatorSimSwitch(new ElevatorIOTalonFX()));
    private final Arm disarm = new Arm(RobotSim.armSimSwitch(new ArmIOSparkEncoderTalonFX()));

    public ArmSafetyData getArmSafetyData() {
        return isInDanger ? ArmConstants.SAFETY_RANGE : ArmConstants.UNSAFE_RANGE;
    }

    public Command sysIDCommand(BooleanSupplier testArm, BooleanSupplier dynamic, Supplier<Direction> direction) {
        return testArm.getAsBoolean() ? summit.sysIDTest(dynamic.getAsBoolean(), direction.get())
                : disarm.sysIDTest(dynamic.getAsBoolean(), direction.get());
    }

    public double safeClampRangeDeg(double inputDeg) {
        return Units.radiansToDegrees(safeClampRange(Units.degreesToRadians(inputDeg)));
    }

    public double safeClampRange(double inputRad) {
        isInDanger = (summit.getCurrentPosition() < ElevatorConstants.SAFETY_THRESHOLD
                || summit.getTargetPosition() < ElevatorConstants.SAFETY_THRESHOLD);
        double[] range = getArmSafetyData().range();
        return MathUtil.clamp(inputRad, range[0], range[1]);
    }

    public void resetArmPid() {
        disarm.pidReset();
    }

    public Command runToPositionCommand(Supplier<ArmisticePositions> position) {
        return Commands.runOnce(() -> {
            elevatorTargetInches = position.get().elevatorPositionInches;
            armTargetRad = Units.degreesToRadians(position.get().armPositionDeg);
        }, summit, disarm).alongWith(Commands.waitUntil(armAndElevatorAtTarget()));
    }

    public Command nudgeCommand(double elevatorInches, double armDegrees) {
        return Commands.runOnce(() -> {
            elevatorTargetInches += elevatorInches;
            armTargetRad += Units.degreesToRadians(armDegrees);
        }, summit, disarm);
    }

    public BooleanSupplier armAndElevatorAtTarget() {
        return () -> disarm.atTargetPosition().getAsBoolean() && summit.atTargetPosition().getAsBoolean();
    }

    public SimData getSimData() {
        return new SimData(summit.getSimPos(), disarm.getSimAngle());
    }

    public boolean isInDanger() {
        return isInDanger;
    }

    public BooleanSupplier isInDangerSupplier() {
        return this::isInDanger;
    }

    public void setInDanger(boolean isInDanger) {
        this.isInDanger = isInDanger;
    }

    @Override
    public void periodic() {
        disarm.runToPosition(safeClampRange(armTargetRad));
        summit.runToPosition(elevatorTargetInches);
    }
}
