package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;

public class AlgaeJoint extends SmartMotorControllerGenericSubsystem {

    private static final String NAMESPACE_NAME = "storage";

    private final SparkWrapper spark;

    public AlgaeJoint(SparkWrapper spark) {
        super(NAMESPACE_NAME, spark);
        this.spark = spark;
    }
}
