package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.util.MotorData;
import frc.robot.util.SysIDUtil;
import frc.robot.util.MathUtil;

public class Arm extends SubsystemBase {
    private final SparkMax spark;
    private final AbsoluteEncoder absEncoder;
    private final TalonFX motor;
    private boolean hasAlgae;
    private final Elevator parentElevator;
    private final Slot0Configs pidConfig;
    private final PositionVoltage pid;
    private final ProfiledPIDController pidController;
    // private final MutableElevatorFeedforward eleFF;
    private final ArmFeedforward armFF;
    private double targetVBus, targetPositionRad;

    // private final DigitalInput di;
    // private final DutyCycleEncoder encoder;

    private final SysIdRoutine.Mechanism sysIDMech;
    private final SysIdRoutine.Config sysIDConfig;
    private final SysIdRoutine sysid;

    private double lastPosition = 0;// , encoderVelocity = 0;

    private ArmStates state;

    private boolean isInDanger = false;

    private double encoderPosition, encoderVelocity;

    private static final double PI_1_2 = 0.5 * Math.PI;
    private static final double PI_3_2 = 1.5 * Math.PI;
    private static final double PI_2 = 2 * Math.PI;
    private static final double PI_7_4 = 7 * Math.PI / 4;
    private double reefVolt = 0;

    private static final class ArmSafetyData {
        public ArmSafetyData(double[] range, boolean enableContinuousInput) {
            this.range = range;
            this.enableContinuousInput = enableContinuousInput;
        }

        public double[] range;
        public boolean enableContinuousInput;
    }

    private static final ArmSafetyData UNSAFE_RANGE = new ArmSafetyData(new double[] { 0, PI_7_4 }, false);
    private static final ArmSafetyData SAFE_RANGE = new ArmSafetyData(new double[] { PI_1_2, PI_7_4 }, false);

    public Arm(Elevator elevator) {
        spark = new SparkMax(9, MotorType.kBrushless);
        spark.configure(new SparkMaxConfig().apply(new AbsoluteEncoderConfig().inverted(true)), null, null);
        absEncoder = spark.getAbsoluteEncoder();
        motor = new TalonFX(10);
        motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));//Brake
        pidController = new ProfiledPIDController(3.0, 0.0, 0.0,
                new TrapezoidProfile.Constraints(Math.PI * 2.0, 4 * Math.PI));
        pidController.enableContinuousInput(0.0, 2 * Math.PI);
        pidController.disableContinuousInput();

        // eleFF = new MutableElevatorFeedforward(0, 0, 0, 0);
        armFF = new ArmFeedforward(0.0875, 0.375, 0);

        hasAlgae = false;
        parentElevator = elevator;
        pid = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
        // use Elevator_Static because angle is already compensated for
        pidConfig = new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0)
                .withGravityType(GravityTypeValue.Elevator_Static);
        state = ArmStates.OFF;

        sysIDConfig = new SysIdRoutine.Config(null, null, null, SysIDUtil::logSysIdState);
        sysIDMech = new SysIdRoutine.Mechanism(v -> {
            motor.setControl(new VoltageOut(v));
        }, null, this);

        sysid = new SysIdRoutine(sysIDConfig, sysIDMech);
        encoderPosition = absEncoder.getPosition();
        pidController.reset(getEncoderPositionRad());
    }

    private static final class ArmConstants {
        public static final double ARM_LENGTH_METRES = 0;
        public static final double ARM_MASS_KG = 0;
        public static final double CG = ARM_LENGTH_METRES / 2; // uniform density
        public static final double GEAR_RATIO = 0.03333333333333;
        public static final MotorData motorType = MotorData.KRAKEN_X60_FOC;
    }

    public Command quasiStaticTest(Direction direction) {
        return sysid.quasistatic(direction);
    }

    public Command dynamicTest(Direction direction) {
        return sysid.dynamic(direction);
    }

    public static enum ArmStates {
        OFF,
        FORWARD,
        BACK,
        POSITION,
        VOLTAGE
    }

    public void toggleBrake() {

    }

    public final double getEncoderPositionRad() {
        var rot = encoderPosition - 0.9145;
        rot = rot > 0 ? rot : 1 + rot;
        return rot * 2 * Math.PI;
    }

    public final double getArmAngleRad() {
        var rad = getEncoderPositionRad() - PI_1_2;
        rad = rad > 0 ? rad : 2 * Math.PI + rad;
        return rad;
    }

    public Command runMotorCommand(double vbus) {
        return runOnce(() -> {
            state = vbus == 0 ? ArmStates.OFF : vbus > 0 ? ArmStates.FORWARD : ArmStates.BACK;
            targetVBus = vbus;
        });
    }

    public Command runToPositionCommand(double positionRad) {
        return runOnce(() -> {
            targetPositionRad = positionRad;
            state = ArmStates.POSITION;
        });
    }

    public Command reefVoltShift(double change) {
        return runOnce(() -> {
            reefVolt += change;
            state = ArmStates.VOLTAGE;
        });
    }

    /**
     * Find the torque due to gravity the arm applies on the pivot shaft
     * 
     * @param hasAlgae Whether the arm has algae in it
     * @param angleRad the angle of the arm
     * @return The arm torque (Nm)
     */
    private double armTorqueGravityNM(boolean hasAlgae, double angleRad) {
        // Arm torque assuming no algae and uniform density
        // where L = arm length, m = arm mass, g = apparent gravitational acceleration
        // (g + elevator accel)
        // T = (1/2)L * mg * sin(armAngle)
        // use effective gravity to compensate for elevator carriage acceleration
        double baseTau = ArmConstants.CG * ArmConstants.ARM_MASS_KG * (9.80665 + parentElevator.getAccelaration())
                * Math.sin(angleRad);
        if (hasAlgae) {
            // Arm torque with algae
            // r_a is radius of algae (8.125 in but in metres), m_a is mass of algae (1.5
            // lbs but in metres)
            // T_b + (L + r_a) m_a * g * sin(armAngle)
            // Assume algae is point mass that is radius of algae away from end of arm
            // Value for weight of algae is approximate due to no explicit size in manual
            // (~1.5 lbs per this post:
            // https://www.chiefdelphi.com/t/reefscape-rule-questions/478346/88?u=7dblackhole)
            return baseTau + ((ArmConstants.ARM_LENGTH_METRES + Constants.ALGAE_RADIUS_M)
                    * (9.80665 + parentElevator.getAccelaration())
                    * Constants.ALGAE_WEIGHT_KG * Math.sin(angleRad));
        }

        return baseTau;
    }

    /**
     * <p>
     * Get the voltage required to counteract the gravitational torque applied on
     * the motor shaft.
     * </p>
     * <p>
     * Used to calculate the pid controller kG gain
     * </p>
     * 
     * @return the voltage (volts)
     */
    private double armGravityFF() {
        return ArmConstants.motorType.getVoltage(
                armTorqueGravityNM(hasAlgae, getEncoderPositionRad()) * ArmConstants.GEAR_RATIO,
                motor.getVelocity().getValueAsDouble() * PI_2);
    }

    // private void setContinuousInput() {
    //     if (getArmSafety().enableContinuousInput) {
    //         var range = getArmSafety().range;
    //         pidController.enableContinuousInput(range[0], range[1]);
    //     } else
    //         pidController.disableContinuousInput();
    // }

    private double safeRangeClamp(double value) {
        var range = getArmSafety().range;
        return MathUtil.clamp(value, range[0], range[1]);
    }

    public ArmSafetyData getArmSafety() {
        return isInDanger ? SAFE_RANGE : UNSAFE_RANGE;
    }

    public boolean isInDanger() {
        return isInDanger;
    }

    public void setInDanger(boolean isInDanger) {
        this.isInDanger = isInDanger;
    }

    public void pidReset() {
        pidController.reset(getEncoderPositionRad());
    }

    public double getDegrees(double value) {
        return 360 - Units.radiansToDegrees(value);
    }

    @Override
    public void periodic() {
        encoderPosition = absEncoder.getPosition();
        encoderVelocity = absEncoder.getVelocity();

        switch (state) {
            case OFF:
                motor.set(0);
                break;
            case FORWARD:
            case BACK:
                motor.set(targetVBus);
                break;
            case POSITION:
                motor.setVoltage(pidController.calculate(getEncoderPositionRad(), safeRangeClamp(targetPositionRad))
                        + armFF.calculate(getArmAngleRad(), pidController.getSetpoint().velocity));
                break;
            case VOLTAGE:
                motor.setVoltage(reefVolt);
                break;
            default:
                break;
        }

        SmartDashboard.putNumber("Absolute Encoder", getEncoderPositionRad());
        SmartDashboard.putNumber("Voltage", reefVolt);
        SmartDashboard.putNumber("Abs encoder arm angle Rad", getArmAngleRad());
        SmartDashboard.putNumber("Raw ABS Encoder", encoderPosition);
        SmartDashboard.putNumber("ArmAmps", motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("ArmVolts", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("PID Target", pidController.getSetpoint().position);
        SmartDashboard.putNumber("PID Velocity", pidController.getSetpoint().velocity);
        SmartDashboard.putBoolean("Arm Danger", isInDanger);
        SmartDashboard.putNumber("degrees", getDegrees(getArmAngleRad()));
        
    }
}
