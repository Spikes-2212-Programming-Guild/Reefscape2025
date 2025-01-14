package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;

public class Gripper extends MotoredGenericSubsystem {

    public Gripper(String namespaceName, SparkMax motor) {
        super(namespaceName, motor);
    }

    public boolean hasAlgae() {
        return false;
    }

    @Override
    public void configureDashboard() {
    }
}
