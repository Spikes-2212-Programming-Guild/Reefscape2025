package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;
import frc.robot.commands.MoveToHeight;
import frc.robot.util.SparkWrapper;

import java.util.function.Supplier;

public class Elevator extends SmartMotorControllerGenericSubsystem {

    public enum ElevatorLevel {

        BOTTOM(0), PROCESSOR(-1), FEEDER(0.16), L1(0), L2(0.37), L3(0.7), L4(-1), TOP(0.72);

        public final double height;

        ElevatorLevel(double height) {
            this.height = height;
        }
    }

    public static final double ELEVATOR_FORWARD_SPEED = 0.175;
    public static final double ELEVATOR_BACKWARD_SPEED = -0.175;

    private static final String NAMESPACE_NAME = "elevator";

    private static final double GEAR_RATIO = (12 / 50.0) * (16 / 50.0) * (35 / 50.0) / 2;
    private static final double HEIGHT_PER_ROTATION = GEAR_RATIO * 2 * Math.PI * 1.625 * 0.0254;
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
        super(namespaceName, master);
        this.master = master;
        slave.follow(master);
        this.topLimit = topLimit;
        this.bottomLimit = bottomLimit;
        master.setPositionConversionFactor(HEIGHT_PER_ROTATION);
        master.setVelocityConversionFactor(HEIGHT_PER_ROTATION / SECONDS_IN_MINUTES);
        master.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        slave.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
//        master.applyConfiguration(master.getSparkConfiguration().smartCurrentLimit(40));
        master.applyConfiguration(master.getSparkConfiguration().closedLoopRampRate(0.4));
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
        } else if (atBottom()) {
            master.setPosition(ElevatorLevel.BOTTOM.height);
        }
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("top limit", topLimit::get);
        namespace.putBoolean("bottom limit", bottomLimit::get);
        namespace.putNumber("elevator height", this::getPosition);
        namespace.putNumber("velocity", this::getVelocity);
        Supplier<Double> speed = namespace.addConstantDouble("speed", 0);
        namespace.putCommand("go up", new MoveGenericSubsystem(this, speed));
        namespace.putCommand("go down", new MoveGenericSubsystem(this, ELEVATOR_BACKWARD_SPEED));
        namespace.putCommand("pid", new MoveToHeight(this, speed));
    }
}
