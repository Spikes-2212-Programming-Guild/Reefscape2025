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

    private final DigitalInput minLimit;
    private final DigitalInput maxLimit;

    private static CoralJoint instance;

    public static CoralJoint getInstance() {
        if (instance == null) {
            instance = new CoralJoint(NAMESPACE_NAME,
                    SparkWrapper.createSparkMax(RobotMap.CAN.CORAL_JOINT_SPARK, SparkLowLevel.MotorType.kBrushless),
                    new DigitalInput(RobotMap.CAN.CORAL_JOINT_MIN_LIMIT),
                    new DigitalInput(RobotMap.CAN.CORAL_JOINT_MAX_LIMIT));
        }
        return instance;
    }

    public CoralJoint(String namespaceName, SparkWrapper joint, DigitalInput minLimit, DigitalInput maxLimit) {
        super(namespaceName, joint);
        this.minLimit = minLimit;
        this.maxLimit = maxLimit;
    }

    public boolean canMove(double speed) {
        return (minLimit.get() && speed > 0) || (maxLimit.get() && speed < 0);
    }
}
