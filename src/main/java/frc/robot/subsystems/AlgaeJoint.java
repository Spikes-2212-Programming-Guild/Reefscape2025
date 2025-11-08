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
    private final DigitalInput topLimit;

    private static AlgaeJoint instance;

    public static AlgaeJoint getInstance() {
        if (instance == null) {
            instance = new AlgaeJoint(SparkWrapper.createSparkMax(RobotMap.CAN.ALGAE_JOINT_SPARK,
                    SparkLowLevel.MotorType.kBrushless), new DigitalInput(RobotMap.DIO.ALGAE_TOP_LIMIT));
        }
        return instance;
    }

    private AlgaeJoint(SparkWrapper spark, DigitalInput topLimit) {
        super(NAMESPACE_NAME, spark);
        this.spark = spark;
        this.topLimit = topLimit;
        spark.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        configureDashboard();
    }

    @Override
    public boolean canMove(double speed) {
        return !(speed > 0 && topLimit.get());
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("top limit", topLimit::get);
        namespace.putCommand("rotate to top", new RotateAlgaeJointToTop(this));
        namespace.putCommand("rotate to bottom", new RotateAlgaeJointToBottom(this));
        namespace.putNumber("encoder", spark::getPosition);
    }
}
