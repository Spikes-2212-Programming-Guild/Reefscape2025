package frc.robot.subsystems.district2;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class District2CoralJoint extends SmartMotorControllerGenericSubsystem {

    public enum StoragePose {

        INTAKE(-1), L1(-1), L2(-1), L3(-1), RESTING(-1);

        public final double neededPitch;

        StoragePose(double neededPitch) {
            this.neededPitch = neededPitch;
        }
    }

    public static final double CORAL_JOINT_FORWARD_SPEED = 0.5;
    public static final double CORAL_JOINT_BACKWARD_SPEED = 0.5;

    private static final String NAMESPACE_NAME = "district 2 coral joint";
    private static final double GEAR_RATIO = 1;
    private static final double DEGREES_IN_ROTATIONS = 360;
    private static final double DISTANCE_PER_PULSE = GEAR_RATIO * DEGREES_IN_ROTATIONS;

    private final TalonFXWrapper talonFX;
    private final DigitalInput bottomLimit;

    private static District2CoralJoint instance;

    public static District2CoralJoint getInstance() {
        if (instance == null) {
            instance = new District2CoralJoint(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.CORAL_JOINT_TALON),
                    new DigitalInput(RobotMap.DIO.CORAL_JOINT_BOTTOM_LIMIT));
        }
        return instance;
    }

    private District2CoralJoint(String namespaceName, TalonFXWrapper talonFX, DigitalInput bottomLimit) {
        super(namespaceName, talonFX);
        this.talonFX = talonFX;
        this.bottomLimit = bottomLimit;
        talonFX.setEncoderConversionFactor(DISTANCE_PER_PULSE);
        configureDashboard();
    }

    @Override
    public boolean canMove(double speed) {
        return !(bottomLimit.get() && speed < 0);
    }

    public void calibrateEncoderPosition() {
        if (bottomLimit.get()) {
            talonFX.setPosition(StoragePose.RESTING.neededPitch);
        }
    }

    public void configureDashboard() {
        namespace.putBoolean("bottom limit", bottomLimit::get);
        namespace.putNumber("storage pose", talonFX::getPosition);
    }
}
