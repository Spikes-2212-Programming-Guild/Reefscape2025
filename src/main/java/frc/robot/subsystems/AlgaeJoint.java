package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeJoint extends SmartMotorControllerGenericSubsystem {

    private static final String NAMESPACE_NAME = "storage";

    private final DigitalInput topLimit;
    private final DigitalInput bottomLimit;

    public AlgaeJoint(SparkWrapper spark, DigitalInput topLimit, DigitalInput bottomLimit) {
        super(NAMESPACE_NAME, spark);
        this.topLimit = topLimit;
        this.bottomLimit = bottomLimit;
    }

    public boolean canMove(double speeds) {
        return (speeds > 0 && topLimit.get()) || (speeds < 0 && bottomLimit.get());
    }

    public void moveJoint(double speeds){

    }

    public void stop(){

    }
}
