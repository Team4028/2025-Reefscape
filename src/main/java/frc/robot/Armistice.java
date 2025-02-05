package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.util.RobotSim;

public class Armistice {

    public static enum ArmisticePositions {
        STOW(180, 7),
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

    private final Elevator elevator = new Elevator(RobotSim.elevatorSimSwitch(new ElevatorIOTalonFX()));
    private final Arm disarm = new Arm(RobotSim.armSimSwitch(new ArmIOSparkEncoderTalonFX()));
}
