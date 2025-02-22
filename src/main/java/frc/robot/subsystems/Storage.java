package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import frc.robot.util.SparkWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class Storage extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "storage";

    private final DigitalInput infrared;

    private static Storage instance;

    public static Storage getInstance() {
        if (instance == null) {
            instance = new Storage(NAMESPACE_NAME, SparkWrapper.createSparkMax(RobotMap.CAN.STORAGE_SPARK,
                    SparkLowLevel.MotorType.kBrushless), new DigitalInput(RobotMap.DIO.STORAGE_INFRARED));
        }
        return instance;
    }

    private Storage(String namespaceName, SparkWrapper motor, DigitalInput infrared) {
        super(namespaceName, motor);
        this.infrared = infrared;
        configureDashboard();
    }

    public boolean hasCoral() {
        return infrared.get();
    }

    @Override
    public boolean canMove(double speed) {
        return !(speed > 0 && hasCoral());
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("infrared", infrared::get);
    }
}
