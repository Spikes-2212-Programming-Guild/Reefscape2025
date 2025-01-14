package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;

public class Gripper extends MotoredGenericSubsystem {

    private static final double PULL_SPEED = -0.5;
    private static final double PUSH_SPEED = 0.5;

    public Gripper(String namespaceName, SparkMax motor) {
        super(namespaceName, motor);
    }

    public void intake() {
        apply(PULL_SPEED);
    }

    public void release() {
        apply(PUSH_SPEED);
    }

    public boolean hasAlgae() {
        return false;
    }

    @Override
    public void configureDashboard() {
    }
}
