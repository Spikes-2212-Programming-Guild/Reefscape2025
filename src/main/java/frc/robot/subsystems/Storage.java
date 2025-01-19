package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class Storage extends MotoredGenericSubsystem {

    private final DigitalInput laser;

    public Storage(String namespaceName, SparkMax motor, DigitalInput laser) {
        super(namespaceName, motor);
        this.laser = laser;
    }

    public boolean hasCoral() {
        return laser.get();
    }

    @Override
    public boolean canMove(double speed) {
        return !(speed < 0 && hasCoral());
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("laser", laser::get);
    }
}
