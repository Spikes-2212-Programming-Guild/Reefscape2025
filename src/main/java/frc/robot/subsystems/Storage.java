package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class Storage extends MotoredGenericSubsystem {

    private static final String NAME_SPACE_NAME = "storage";
    private final DigitalInput infrared;

    private static Storage instance;
    public static Storage getInstance() {
        if (instance == null) {
            instance = new Storage(NAME_SPACE_NAME, SparkWrapper.createSparkMax(RobotMap.CAN.STORAGE_SPARK,
                    SparkLowLevel.MotorType.kBrushless), new DigitalInput(RobotMap.DIO.HAS_CORAL));
        }
        return instance;
    }

    public Storage(String namespaceName, SparkWrapper motor, DigitalInput infrared) {
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
