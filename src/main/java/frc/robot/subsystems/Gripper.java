package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class Gripper extends MotoredGenericSubsystem {

    private final DigitalInput limit;

    public Gripper(String namespaceName, SparkMax motor, DigitalInput limit) {
        super(namespaceName, motor);
        this.limit = limit;
    }

    public boolean hasAlgae() {
        return limit.get();
    }

    @Override
    public void configureDashboard() {
    }
}
