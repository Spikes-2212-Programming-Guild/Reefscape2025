package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class CoralJoint extends SmartMotorControllerGenericSubsystem {

    public enum STORAGE_POSE {
        L1(-1), L2(-1), L3(-1), L4(-1);

        public final double neededPitch;

        STORAGE_POSE(double neededPitch) {
            this.neededPitch = neededPitch;
        }

    }
    private final DigitalInput minLimit;
    private final DigitalInput maxLimit;

    private static CoralJoint instance;
    public static CoralJoint getInstance() {
        if (instance == null) {
            instance = new CoralJoint("coral joint", DigitalInput, RobotMap.CAN.MAX_LIMIT,
                    SparkWrapper.createSparkMax(RobotMap.CAN.CORAL_JOINT, SparkLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    public CoralJoint(String namespaceName, DigitalInput minLimit, DigitalInput maxLimit, SparkWrapper joint) {
        super(namespaceName, joint);
        this.minLimit = minLimit;
        this.maxLimit = maxLimit;
    }

    public boolean canMove(double speed) {
        return (minLimit.get() && speed < 0) || (maxLimit.get() && speed > 0);
    }
}
