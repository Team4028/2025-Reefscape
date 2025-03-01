package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmConstants.ArmSafetyData;
import frc.robot.subsystems.arm.ArmIOSparkEncoderTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
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
    private double armTargetRad = ArmisticePositions.STOW.armPositionRad;

    private double armCharVoltage = 0;

    public static enum ArmisticePositions {
        STOW(0.43, 3.6),
        ACQUIRE(0.43, 3.6),
        L2(3.64, 16),
        L3(3.64, 31.6),
        L4(3.411, 35.4),
        ALGAE_AQUIRE_L2(5.7, 8.64),
        ALGAE_AQUIRE_L3(5.8, 18.9),
        LOLLIPOP_ACQUIRE(4.85, 2);

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
    private final Arm disarm = RobotSim.armSimSwitch(new ArmIOSparkEncoderTalonFX());

    public ArmSafetyData getArmSafetyData() {
        return isInDanger ? ArmConstants.SAFETY_RANGE : ArmConstants.UNSAFE_RANGE;
    }

    /** Kills the arm(y) */
    public void orbitalStrike() {
        disarm.runMotor(0);
    }

    public Command sysIDCommandElevator(BooleanSupplier dynamic, Supplier<Direction> direction) {
        return summit.sysIDTest(dynamic.getAsBoolean(), direction.get());
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

    public Command runToPositionCommand(Supplier<ArmisticePositions> position) {
        return runToPositionCommand(position.get());
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

    public BooleanSupplier armAndElevatorAtTarget() {
        return () -> disarm.atTargetPosition().getAsBoolean() && summit.atTargetPosition().getAsBoolean();
    }

    public Arm getArm() {
        return disarm;
    }

    public Elevator getElevator() {
        return summit;
    }

    public Subsystem[] getSubsystems() {
        return new Subsystem[] { disarm, summit };
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
