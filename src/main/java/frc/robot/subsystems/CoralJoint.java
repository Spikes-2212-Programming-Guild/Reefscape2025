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

        INTAKE(-1), PLACEMENT(-1), RESTING(-1);

        public final double neededPitch;

        StoragePose(double neededPitch) {
            this.neededPitch = neededPitch;
        }
    }

    public static final double CORAL_JOINT_FORWARD_SPEED = 0.1;
    public static final double CORAL_JOINT_BACKWARD_SPEED = 0;

    private static final String NAMESPACE_NAME = "coral joint";
    private static final double GEAR_RATIO = 1;
    private static final double DEGREES_IN_ROTATIONS = 360;
    private static final double DISTANCE_PER_PULSE = GEAR_RATIO * DEGREES_IN_ROTATIONS;

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
            talonFX.setPosition(StoragePose.RESTING.neededPitch);
        } else if (bottomLimit.get()) {
            talonFX.setPosition(StoragePose.PLACEMENT.neededPitch);
        }
    }

    public void configureDashboard() {
        namespace.putBoolean("top limit", topLimit::get);
        namespace.putBoolean("bottom limit", bottomLimit::get);
        namespace.putNumber("storage pose", talonFX::getPosition);
    }
}
