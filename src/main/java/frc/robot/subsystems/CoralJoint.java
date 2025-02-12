package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class CoralJoint extends SmartMotorControllerGenericSubsystem {

    public enum STORAGE_POSE {

        INTAKE(-1), PLACEMENT(-1), RESTING(-1);

        public final double neededPitch;

        STORAGE_POSE(double neededPitch) {
            this.neededPitch = neededPitch;
        }
    }

    private static final String NAMESPACE_NAME = "coral joint";
    private static final double GEAR_RATIO = 1;
    private static final double DEGREES_IN_ROTATIONS = 360;
    private static final double DISTANCE_PER_PULSE = GEAR_RATIO * DEGREES_IN_ROTATIONS;
    private static final double SECONDS_IN_MINUTE = 60;

    private final SparkWrapper spark;
    private final DigitalInput topLimit;
    private final DigitalInput bottomLimit;

    private static CoralJoint instance;

    public static CoralJoint getInstance() {
        if (instance == null) {
            instance = new CoralJoint(NAMESPACE_NAME,
                    SparkWrapper.createSparkMax(RobotMap.CAN.CORAL_JOINT_SPARK, SparkLowLevel.MotorType.kBrushless),
                    new DigitalInput(RobotMap.DIO.CORAL_JOINT_TOP_LIMIT),
                    new DigitalInput(RobotMap.DIO.CORAL_JOINT_BOTTOM_LIMIT));
        }
        return instance;
    }

    private CoralJoint(String namespaceName, SparkWrapper spark, DigitalInput topLimit, DigitalInput bottomLimit) {
        super(namespaceName, spark);
        this.spark = spark;
        this.topLimit = topLimit;
        this.bottomLimit = bottomLimit;
        spark.setPositionConversionFactor(DISTANCE_PER_PULSE);
        spark.setVelocityConversionFactor(DISTANCE_PER_PULSE / SECONDS_IN_MINUTE);
        configureDashboard();
    }

    @Override
    public boolean canMove(double speed) {
        return !((bottomLimit.get() && speed < 0) || (topLimit.get() && speed > 0));
    }

    public void calibrateEncoderPosition() {
        if (topLimit.get()) {
            spark.setPosition(STORAGE_POSE.RESTING.neededPitch);
        } else if (bottomLimit.get()) {
            spark.setPosition(STORAGE_POSE.PLACEMENT.neededPitch);
        }
    }

    public void configureDashboard() {
        namespace.putBoolean("top limit", topLimit::get);
        namespace.putBoolean("bottom limit", bottomLimit::get);
        namespace.putNumber("storage pose", spark::getPosition);
    }
}
