package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class Elevator extends SmartMotorControllerGenericSubsystem {

    public enum ElevatorLevel {

        BOTTOM(0), PROCESSOR(0.18), FEEDER(0.55), L1(0.7), L2(1.2), L3(1.6), L4(1.8), TOP(2);

        public final double height;

        ElevatorLevel(double height) {
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
        configureDashboard();
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

    public void calibratePosition() {
        if (atTop()) {
            master.setPosition(ElevatorLevel.TOP.height);
        }
        else if (atBottom()) {
            master.setPosition(ElevatorLevel.BOTTOM.height);
        }
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("top limit", topLimit::get);
        namespace.putBoolean("bottom limit", bottomLimit::get);
        namespace.putNumber("elevator height", getPosition());
    }
}
