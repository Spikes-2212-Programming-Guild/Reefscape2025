package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class AlgaeJoint extends SmartMotorControllerGenericSubsystem {

    private static final String NAMESPACE_NAME = "storage";

    private static AlgaeJoint instance;

    private final DigitalInput topLimit;
    private final DigitalInput bottomLimit;

    public static AlgaeJoint getInstance() {
        if (instance == null){
            instance = new AlgaeJoint(SparkWrapper.createSparkMax(RobotMap.CAN.ALGAE_SPARK,
                    SparkLowLevel.MotorType.kBrushless), new DigitalInput(RobotMap.DIO.TOP_LIMITER),
                    new DigitalInput(RobotMap.DIO.BOTTOM_LIMITER));
        }
        return instance;
    }

    public AlgaeJoint(SparkWrapper spark, DigitalInput topLimit, DigitalInput bottomLimit) {
        super(NAMESPACE_NAME, spark);
        this.topLimit = topLimit;
        this.bottomLimit = bottomLimit;
    }

    @Override
    public boolean canMove(double speed) {
        return (speed < 0 && topLimit.get()) || (speed > 0 && bottomLimit.get());
    }
}
