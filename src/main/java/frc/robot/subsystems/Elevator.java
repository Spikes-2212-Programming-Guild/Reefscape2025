package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class Elevator extends SmartMotorControllerGenericSubsystem {

    public enum ElevatorLevels {
        //@Todo check with mechanics if the processor and feeder require the same heights as other levels
        PROCESSOR(-1), FEEDER(-1), L1(-1), L2(-1), L3(-1), L4(-1);

        public final double height;

        ElevatorLevels(double height) {
            this.height = height;
        }
    }

    private static final String NAMESPACE_NAME = "elevator";

    private static final double GEAR_RATIO = (14 / 50.0) * (16 / 50.0);
    private static final double HEIGHT_PER_ROTATION = 2 / (GEAR_RATIO * 41.2 * Math.PI);
    private static final double SECONDS_IN_MINUTES = 60;

    private final SparkWrapper master;
    private final DigitalInput topLimit;
    private final DigitalInput bottomLimit;

    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator(NAMESPACE_NAME, SparkWrapper.createSparkMax(RobotMap.CAN.ELEVATOR_MASTER_SPARK,
                    SparkLowLevel.MotorType.kBrushless), SparkWrapper.createSparkMax(RobotMap.CAN.ELEVATOR_SLAVE_SPARK,
                    SparkLowLevel.MotorType.kBrushless), new DigitalInput(RobotMap.DIO.ELEVATOR_TOP_LIMIT),
                    new DigitalInput(RobotMap.DIO.ELEVATOR_BOTTOM_LIMIT));
        }
        return instance;
    }

    public Elevator(String namespaceName, SparkWrapper master, SparkWrapper slave, DigitalInput topLimit,
                    DigitalInput bottomLimit) {
        super(namespaceName, master, slave);
        this.master = master;
        this.topLimit = topLimit;
        this.bottomLimit = bottomLimit;
        master.setPositionConversionFactor(HEIGHT_PER_ROTATION);
        master.setVelocityConversionFactor(HEIGHT_PER_ROTATION / SECONDS_IN_MINUTES);
        slave.setInverted(true);
    }

    public double getPosition() {
        return master.getPosition();
    }

    public double getVelocity() {
        return master.getVelocity();
    }

    public boolean atTop() {
        return topLimit.get();
    }

    public boolean atBottom() {
        return bottomLimit.get();
    }

    public boolean canMove(double speed) {
        return !((atTop() && speed > 0) || (atBottom() && speed < 0));
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("top limit", topLimit::get);
        namespace.putBoolean("bottom limit", bottomLimit::get);
    }
}
