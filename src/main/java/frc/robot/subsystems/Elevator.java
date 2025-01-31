package frc.robot.subsystems;

import com.revrobotics.spark.config.EncoderConfig;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator extends SmartMotorControllerGenericSubsystem {

    public enum ElevatorLevels {

        PROCESSOR(-1), FEEDER(-1), L1(-1), L2(-1), L3(-1), L4(-1);

        public final double height;

        ElevatorLevels(double height) {
            this.height = height;
        }
    }

    private static final double GEAR_RATIO = (14 / 50.0) * (16 / 50.0);
    private static final double SPINS_TO_HEIGHT = 2 / (GEAR_RATIO * 41.2 * Math.PI);

    private final SparkWrapper master;

    private final DigitalInput bottomLimit;
    private final DigitalInput topLimit;

    public Elevator(String namespaceName, SparkWrapper master, SparkWrapper slave, DigitalInput bottomLimit,
                    DigitalInput topLimit) {
        super(namespaceName, master, slave);
        this.master = master;
        this.bottomLimit = bottomLimit;
        this.topLimit = topLimit;
        slave.setInverted(true);

        master.setPositionConversionFactor(SPINS_TO_HEIGHT);
    }

    public double getSpeed() {
        return master.getVelocity();
    }

    public double getPosition() {
        return master.getPosition() * SPINS_TO_HEIGHT;
    }

    public boolean isBottom() {
        return bottomLimit.get();
    }

    public boolean isTop() {
        return topLimit.get();
    }

    public boolean canMove(double speed) {
        return !((isBottom() && speed < 0) || (isTop() && speed > 0));
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("bottom limit", bottomLimit::get);
        namespace.putBoolean("top limit", topLimit::get);
    }
}
