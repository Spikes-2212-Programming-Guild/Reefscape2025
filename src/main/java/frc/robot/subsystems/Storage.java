package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class Storage extends MotoredGenericSubsystem {

    private final DigitalInput infrared;

    public Storage(String namespaceName, SparkMax motor, DigitalInput infrared) {
        super(namespaceName, motor);
        this.infrared = infrared;
    }

    public boolean hasCoral() {
        return infrared.get();
    }

    @Override
    public boolean canMove(double speed) {
        return !(speed < 0 && hasCoral());
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("infrared", infrared::get);
    }
}
