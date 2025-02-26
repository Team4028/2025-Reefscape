package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private double armTargetRad = ArmisticePositions.STOW.armPositionRad;

    @AutoLogOutput
    private double armCharVoltage = 0;

    public static enum ArmisticePositions {
        STOW(0.43, 3.6),
        ACQUIRE(0.74, 4.6),
        ACQUIRE_BLOCKED(0.94, 2.6),
        L2(3.9, 9),
        L3(3.9, 24.5),
        L4(3.825, 51.01),
        ALGAE_AQUIRE_L2(5.33, 8.64),
        LOLLIPOP_ACQUIRE(4.85, 2),
        BARGE(1.45, 50.1);

        public double armPositionRad;
        public double elevatorPositionInches;

        private ArmisticePositions(double armPositionRad, double elevatorPositionInches) {
            this.armPositionRad = armPositionRad;
            this.elevatorPositionInches = elevatorPositionInches;
        }
    }

    public Armistice() {
        NamedCommands.registerCommand("L4 Score", runToPositionCommand(() -> ArmisticePositions.L4));
        NamedCommands.registerCommand("Stow", runToPositionCommand(() -> ArmisticePositions.STOW));
        NamedCommands.registerCommand("Stow No Wait", runToPositionNoWait(() -> ArmisticePositions.STOW));
        NamedCommands.registerCommand("Acquire Pos", runToPositionCommand(() -> ArmisticePositions.ACQUIRE));
        NamedCommands.registerCommand("L3 Score", runToPositionCommand(() -> ArmisticePositions.L3));
    }

    private final Elevator summit = RobotSim.elevatorSimSwitch(new ElevatorIOTalonFX());


    //Mag Arm is exactly the same as disarm, but uses a can encoder instead of spark. 
  
    private final Arm disarm = RobotSim.armSimSwitch(new ArmIOCanEncoderTalonFX());


    public void getCanMagPosition() {
        disarm.getCanMagPosition();;
    }

    public ArmSafetyData getArmSafetyData() {
        return isInDanger ? ArmConstants.SAFETY_RANGE : ArmConstants.UNSAFE_RANGE;
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

    public Command runToPositionCommand(Supplier<ArmisticePositions> position) {
        return Commands.runOnce(() -> {
            elevatorTargetInches = position.get().elevatorPositionInches;
            armTargetRad = position.get().armPositionRad;
        }, summit, disarm).alongWith(Commands.waitUntil(armAndElevatorAtTarget()));
    }

    public Command runToPositionNoWait(Supplier<ArmisticePositions> position) {
        return Commands.runOnce(() -> {
            elevatorTargetInches = position.get().elevatorPositionInches;
            armTargetRad = position.get().armPositionRad;
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
    public double getElevatorPosition() {
        return summit.getCurrentPosition();
    }

    public SubsystemBase[] getSubsystems() {
        return new SubsystemBase[]{disarm, summit};
    }

    @Override
    public void periodic() {
        // disarm.runToPosition(safeClampRange(armTargetRad)); 
        summit.runToPosition(elevatorTargetInches);
    }
}
