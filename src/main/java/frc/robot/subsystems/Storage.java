package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;

public class Storage extends MotoredGenericSubsystem {

    private static final double PULL_SPEED = -0.5;
    private static final double PUSH_SPEED = 0.5;

    public Storage(String namespaceName, SparkMax motor) {
        super(namespaceName, motor);
    }

    public void intake() {
        apply(PULL_SPEED);
    }

    public void release() {
        apply(PUSH_SPEED);
    }

    public boolean hasCoral() {
        return false;
    }

    @Override
    public void configureDashboard() {
    }
}
