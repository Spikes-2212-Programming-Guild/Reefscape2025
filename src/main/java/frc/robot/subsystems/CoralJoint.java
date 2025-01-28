package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SmartMotorController;
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralJoint  extends SmartMotorControllerGenericSubsystem {

    public enum STORAGE_POSE {
        L1(-1), L2(-1), L3(-1), L4(-1);

        public final double neededPitch;

        STORAGE_POSE(double neededPitch) {
            this.neededPitch = neededPitch;
        }
    }

    private final DigitalInput minLimit;
    private final DigitalInput maxLimit;

    private final SparkMax joint;

    public CoralJoint(String namespaceName, DigitalInput minLimit, DigitalInput maxLimit, SparkMax joint) {
        super(namespaceName, (SmartMotorController) joint);
        this.minLimit = minLimit;
        this.maxLimit = maxLimit;
        this.joint = joint;
    }

    public boolean canMove(double speed) {
        return (minLimit.get() && speed < 0) || (maxLimit.get() && speed > 0);
    }
}
