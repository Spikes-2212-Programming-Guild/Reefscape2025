package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkGenericSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator extends SparkGenericSubsystem {

    public enum ReefLevel {
        L1(-1), L2(-1), L3(-1), L4(-1);

        public final double height;

        ReefLevel(double height) {
            this.height = height;
        }
    }

    private static final double GEAR_RATIO = (14 / 50.0) * (16 / 50.0);
    private static final double SPINS_TO_HEIGHT = 2 / (GEAR_RATIO * 41.2 * Math.PI);

    private final DigitalInput minLimit;
    private final DigitalInput maxLimit;
    private final DigitalInput hallEffect;

    public Elevator(String namespaceName, SparkMax master, SparkMax slave, DigitalInput minLimit,
                    DigitalInput maxLimit, DigitalInput hallEffect) {
        super(namespaceName, master, slave);
        this.minLimit = minLimit;
        this.maxLimit = maxLimit;
        this.hallEffect = hallEffect;
        SparkMaxConfig slaveConfig = new SparkMaxConfig();
        slaveConfig.inverted(true);
        slave.configure(slaveConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(SPINS_TO_HEIGHT);
    }

    public double getSpeed() {
        return master.getEncoder().getVelocity();
    }

    public double getPosition() {
        return master.getEncoder().getPosition() * SPINS_TO_HEIGHT;
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
        namespace.putBoolean("hallEffectLvl2", hallEffect::get);
    }
}
