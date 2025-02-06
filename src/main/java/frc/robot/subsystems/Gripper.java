package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class Gripper extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "gripper";

    private final DigitalInput limit;

    private static Gripper instance;

    public static Gripper getInstance() {
        if (instance == null) {
            instance = new Gripper(NAMESPACE_NAME, new WPI_TalonSRX(RobotMap.CAN.GRIPPER_TALON),
                    new DigitalInput(RobotMap.DIO.GRIPPER_LIMIT));
        }
        return instance;
    }

    private Gripper(String namespaceName, WPI_TalonSRX talon, DigitalInput limit) {
        super(namespaceName, talon);
        this.limit = limit;
        talon.setNeutralMode(NeutralMode.Brake);
        configureDashboard();
    }

    public boolean hasAlgae() {
        return limit.get();
    }

    @Override
    public boolean canMove(double speed) {
        return !(speed > 0 && hasAlgae());
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("algae present", limit::get);
    }
}
