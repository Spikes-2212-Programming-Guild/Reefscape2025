package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator extends SparkGenericSubsystem {

    private final DigitalInput minLimit;
    private final DigitalInput maxLimit;

    public Elevator(String namespaceName, SparkMax master, SparkMax slave, DigitalInput minLimit, DigitalInput maxLimit) {
        super(namespaceName, master, slave);
        this.minLimit = minLimit;
        this.maxLimit = maxLimit;
        SparkMaxConfig slaveConfig = new SparkMaxConfig();
        slaveConfig.inverted(true);
        slave.configure(slaveConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    public boolean isMin() {
        return minLimit.get();
    }

    public boolean isMax() {
        return maxLimit.get();
    }

    public boolean canMove(double speed) {
        return !((isMin() && speed < 0) || (isMax() && speed > 0));
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("min limit", minLimit::get);
        namespace.putBoolean("max limit", maxLimit::get);
    }
}
