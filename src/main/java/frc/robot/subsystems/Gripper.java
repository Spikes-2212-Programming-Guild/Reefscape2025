package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class Gripper extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "gripper";

    private final DigitalInput limit;

    private static Gripper instance;

    public static Gripper getInstance() {
        if (instance == null) {
            instance = new Gripper(NAMESPACE_NAME, SparkWrapper.createSparkMax(RobotMap.CAN.GRIPPER_SPARK,
                    SparkLowLevel.MotorType.kBrushless), new DigitalInput(RobotMap.DIO.GRIPPER_LIMIT));
        }
        return instance;
    }

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
