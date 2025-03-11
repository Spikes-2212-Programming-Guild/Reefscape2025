package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.ReleaseCoral;
import frc.robot.util.SparkWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class Storage extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "storage";
    private static final int CURRENT_LIMIT = 40;
    private static final int CORAL_CURRENT_THRESHOLD = 15;

    private final SparkWrapper spark;
    private final DigitalInput infrared;

    private boolean running;

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
        spark = motor;
//        motor.applyConfiguration(motor.getSparkConfiguration().smartCurrentLimit(CURRENT_LIMIT));
        configureDashboard();
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public boolean hasCoral() {
//        return true;
        return !infrared.get();
    }

    public double getCurrent() {
        return spark.getCurrent();
    }

    public void intake() {
        motorController.set(0.5);
    }

    public boolean isRunning() {
        return running;
    }

    public void setRunning(boolean running) {
        this.running = running;
    }

    @Override
    public boolean canMove(double speed) {
//        return !(speed > 0 && hasCoral());
        return !(speed > 0 && getCurrent() > CORAL_CURRENT_THRESHOLD);
//        return true;
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("infrared", infrared::get);
        namespace.putCommand("intake coral", new IntakeCoral(this));
        namespace.putCommand("release coral", new ReleaseCoral(this));
        namespace.putBoolean("can move", () -> canMove(getSpeed()));
        namespace.putCommand("move", new InstantCommand(() -> motorController.set(0.2)));
        namespace.putNumber("current", this::getCurrent);
    }
}
