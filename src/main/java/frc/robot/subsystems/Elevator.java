package frc.robot.subsystems;

import com.revrobotics.spark.config.EncoderConfig;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator extends SmartMotorControllerGenericSubsystem {

    public enum ElevatorLevels {
        processor(-1), feeder(-1), L1(-1), L2(-1), L3(-1), L4(-1);

        public final double height;

        ElevatorLevels(double height) {
            this.height = height;
        }
    }

    private static final double GEAR_RATIO = (14 / 50.0) * (16 / 50.0);
    private static final double SPINS_TO_HEIGHT = 2 / (GEAR_RATIO * 41.2 * Math.PI);

    private final SparkWrapper master;

    private final DigitalInput minLimit;
    private final DigitalInput maxLimit;

    public Elevator(String namespaceName, SparkWrapper master, SparkWrapper slave, DigitalInput minLimit,
                    DigitalInput maxLimit) {
        super(namespaceName, master, slave);
        this.master = master;
        this.minLimit = minLimit;
        this.maxLimit = maxLimit;
        slave.setInverted(true);

        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(SPINS_TO_HEIGHT);
    }

    public double getSpeed() {
        return master.getVelocity();
    }

    public double getPosition() {
        return master.getPosition() * SPINS_TO_HEIGHT;
    }

    public boolean isMin() {
        return minLimit.get();
    }

    public boolean isMax() {
        return maxLimit.get();
    }

    public boolean isInHeight() {return false;}

    public boolean canMove(double speed) {
        return !((isMin() && speed < 0) || (isMax() && speed > 0));
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("min limit", minLimit::get);
        namespace.putBoolean("max limit", maxLimit::get);
    }
}
