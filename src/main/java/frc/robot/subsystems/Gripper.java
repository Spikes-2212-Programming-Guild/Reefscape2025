package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import edu.wpi.first.wpilibj.DigitalInput;

public class Gripper extends MotoredGenericSubsystem {

    private final DigitalInput limit;

    public Gripper(String namespaceName, SparkWrapper motor, DigitalInput limit) {
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
        namespace.putBoolean("algae present", limit::get);
    }
}
