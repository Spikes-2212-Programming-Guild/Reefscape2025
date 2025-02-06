package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class Gripper extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "gripper";
    private static WPI_VictorSPX WPI_VictorSPX;

    private final DigitalInput limit;

    private static Gripper instance;

    public static Gripper getInstance() {
        if (instance == null) {
            instance = new Gripper(NAMESPACE_NAME, WPI_VictorSPX = new WPI_VictorSPX(RobotMap.CAN.GRIPPER_VICTOR),
                    new DigitalInput(RobotMap.DIO.GRIPPER_LIMIT));
        }
        return instance;
    }

    public Gripper(String namespaceName, WPI_VictorSPX motor, DigitalInput limit) {
        super(namespaceName, motor);
        this.limit = limit;
        configureDashboard();
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
