package frc.robot.subsystems.limelight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class LimelightConstants {
    public static final double STD_DEV_POSE_DIFF_THRESHOLD = 2.0;
    public static final Vector<N3> GOOD_STD_DEVS = VecBuilder.fill(0.1, 0.1, Double.MAX_VALUE);
}
