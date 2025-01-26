package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;

public class CoralJoint  extends SparkGenericSubsystem {

    public enum STORAGE_POSE {
        L1(-1), L2(-1), L3(-1), L4(-1);

        public final double neededPitch;

        STORAGE_POSE(double neededPitch) {
            this.neededPitch = neededPitch;
        }
    }

    public CoralJoint(String namespaceName, SparkMax joint) {
        super(namespaceName, joint);
    }
}