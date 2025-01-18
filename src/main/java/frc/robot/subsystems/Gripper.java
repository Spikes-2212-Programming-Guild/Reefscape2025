package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.DigitalInput;

import java.util.function.Supplier;

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
    public boolean canMove(double speed) {
        return !(speed < 0 && hasAlgae());
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("limit", limit::get);
    }
}
