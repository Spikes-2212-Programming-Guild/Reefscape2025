package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;

public class Storage extends MotoredGenericSubsystem {

    public Storage(String namespaceName, SparkMax motor) {
        super(namespaceName, motor);
    }

    public boolean hasCoral() {
        return false;
    }

    @Override
    public void configureDashboard() {
    }
}
