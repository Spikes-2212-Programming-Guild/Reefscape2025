package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;

public class Storage extends MotoredGenericSubsystem {

    private static final double INTAKE_SPEED = -0.5;
    private static final double RELEASE_SPEED = 0.5;

    public Storage(String namespaceName, SparkMax motor) {
        super(namespaceName, motor);
    }

    public void intake() {
        apply(INTAKE_SPEED);
    }

    public void release() {
        apply(RELEASE_SPEED);
    }

    public boolean hasCoral() {
        return false;
    }

    @Override
    public void configureDashboard() {
    }
}
