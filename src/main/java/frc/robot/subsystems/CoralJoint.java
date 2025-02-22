package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class CoralJoint extends SmartMotorControllerGenericSubsystem {

    public enum StoragePose {

        INTAKE(-1), L1(-1), L2(-1), L3(-1), RESTING(-1);

        public final double neededPitch;

        StoragePose(double neededPitch) {
            this.neededPitch = neededPitch;
        }
    }

    public static final double CORAL_JOINT_FORWARD_SPEED = 0.175;
    public static final double CORAL_JOINT_BACKWARD_SPEED = -0.175;

    private static final String NAMESPACE_NAME = "coral joint";
    private static final double GEAR_RATIO = 1;
    private static final double DEGREES_IN_ROTATIONS = 360;
    private static final double DISTANCE_PER_PULSE = GEAR_RATIO * DEGREES_IN_ROTATIONS;
    private static final double SECONDS_IN_MINUTE = 60;

    private final TalonFXWrapper talonFX;
    private final DigitalInput topLimit;
    private final DigitalInput bottomLimit;

    private static CoralJoint instance;

    public static CoralJoint getInstance() {
        if (instance == null) {
            instance = new CoralJoint(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.CORAL_JOINT_TALON),
                    new DigitalInput(RobotMap.DIO.CORAL_JOINT_TOP_LIMIT),
                    new DigitalInput(RobotMap.DIO.CORAL_JOINT_BOTTOM_LIMIT));
        }
        return instance;
    }

    private CoralJoint(String namespaceName, TalonFXWrapper talonFX, DigitalInput topLimit, DigitalInput bottomLimit) {
        super(namespaceName, talonFX);
        this.talonFX = talonFX;
        this.topLimit = topLimit;
        this.bottomLimit = bottomLimit;
        talonFX.setEncoderConversionFactor(DISTANCE_PER_PULSE);
        configureDashboard();
    }

    @Override
    public boolean canMove(double speed) {
        return !((bottomLimit.get() && speed < 0) || (topLimit.get() && speed > 0));
    }

    public void calibrateEncoderPosition() {
        if (topLimit.get()) {
            talonFX.setPosition(STORAGE_POSE.L3.neededPitch);
        } else if (bottomLimit.get()) {
            talonFX.setPosition(STORAGE_POSE.RESTING.neededPitch);
        }
    }

    public void configureDashboard() {
        namespace.putBoolean("top limit", topLimit::get);
        namespace.putBoolean("bottom limit", bottomLimit::get);
        namespace.putNumber("storage pose", talonFX::getPosition);
    }
}
