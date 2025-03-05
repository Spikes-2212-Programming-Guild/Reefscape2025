package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;
import frc.robot.commands.RotateAlgaeJointToBottom;
import frc.robot.commands.RotateAlgaeJointToTop;
import frc.robot.util.SparkWrapper;

public class AlgaeJoint extends SmartMotorControllerGenericSubsystem {

    public static final double STABILIZATION_SPEED = 0.1;

    private static final String NAMESPACE_NAME = "storage";

    private final SparkWrapper spark;

    private static AlgaeJoint instance;

    public static AlgaeJoint getInstance() {
        if (instance == null) {
            instance = new AlgaeJoint(SparkWrapper.createSparkMax(RobotMap.CAN.ALGAE_JOINT_SPARK,
                    SparkLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    private AlgaeJoint(SparkWrapper spark) {
        super(NAMESPACE_NAME, spark);
        this.spark = spark;
        spark.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        configureDashboard();
    }

    @Override
    public void configureDashboard() {
        namespace.putCommand("rotate to top", new RotateAlgaeJointToTop(this));
        namespace.putCommand("rotate to bottom", new RotateAlgaeJointToBottom(this));
        namespace.putNumber("encoder", spark::getPosition);
    }
}
