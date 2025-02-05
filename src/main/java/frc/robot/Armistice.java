package frc.robot;

import java.util.function.BooleanSupplier;

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

public class Armistice {

    public static final record SimData(double elevatorPositionMeters, double armPositionRadians) {
    }

    private boolean isInDanger = true;

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
        NamedCommands.registerCommand("L4 Score", runToPositionCommand(ArmisticePositions.L4));
        NamedCommands.registerCommand("Stow", runToPositionCommand(ArmisticePositions.STOW));
        NamedCommands.registerCommand("L3 Score", runToPositionCommand(ArmisticePositions.L3));
        NamedCommands.registerCommand("Stow Arm",
                disarm.runToPositionCommand(Units.degreesToRadians(ArmisticePositions.STOW.armPositionDeg)));
    }

    // could rename to "summit" (ex. peace summit)
    private final Elevator elevator = new Elevator(RobotSim.elevatorSimSwitch(new ElevatorIOTalonFX()));
    private final Arm disarm = new Arm(RobotSim.armSimSwitch(new ArmIOSparkEncoderTalonFX()), this::safeClampRange);

    public ArmSafetyData getArmSafetyData() {
        return isInDanger ? ArmConstants.SAFETY_RANGE : ArmConstants.UNSAFE_RANGE;
    }

    public Command sysIDCommand(boolean testArm, boolean dynamic, Direction direction) {
        return testArm ? elevator.sysIDTest(dynamic, direction) : disarm.sysIDTest(dynamic, direction);
    }

    public double safeClampRange(double inputDeg) {
        double inputRad = Units.degreesToRadians(inputDeg);
        isInDanger = (elevator.getCurrentPosition() < ElevatorConstants.SAFETY_THRESHOLD
                || elevator.getTargetPosition() < ElevatorConstants.SAFETY_THRESHOLD);
        double[] range = getArmSafetyData().range();
        return Units.radiansToDegrees(MathUtil.clamp(inputRad, range[0], range[1]));
    }

    public void resetArmPid() {
        disarm.pidReset();
    }

    public Command runToPositionCommand(ArmisticePositions position) {
        return elevator.runToPositionCommand(position.elevatorPositionInches)
                .alongWith(disarm.runToPositionCommand(Units.degreesToRadians(safeClampRange(position.armPositionDeg))))
                .alongWith(Commands.waitUntil(armAndElevatorAtTarget()));
    }

    public Command nudgeCommand(double elevatorInches, double armDegrees) {
        return elevator.nudgeCommand(elevatorInches).alongWith(disarm.nudgeCommand(Units.degreesToRadians(armDegrees)));
    }

    public BooleanSupplier armAndElevatorAtTarget() {
        return () -> disarm.atTargetPosition().getAsBoolean() && elevator.atTargetPosition().getAsBoolean();
    }

    public SimData getSimData() {
        return new SimData(elevator.getSimPos(), disarm.getSimAngle());
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
}
