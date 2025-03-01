package frc.robot.subsystems.district2;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystemWithPID;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotMap;
import frc.robot.commands.district2.District2RotateStorage;

import java.util.function.Supplier;

public class District2CoralJoint extends SmartMotorControllerGenericSubsystem {

    public enum StoragePose {

        INTAKE(0.06 * DEGREES_IN_ROTATIONS), L1(-0.075 * DEGREES_IN_ROTATIONS), L2(-0.018 * DEGREES_IN_ROTATIONS),
        L3(0.13 * DEGREES_IN_ROTATIONS), RESTING(-0.2075 * DEGREES_IN_ROTATIONS);
//        RESTING(0.0);

        public final double neededPitch;

        StoragePose(double neededPitch) {
            this.neededPitch = neededPitch;
        }
    }

    PIDSettings pidSettings = namespace.addPIDNamespace("dis 2 coral joint");
    FeedForwardSettings feedForwardSettings = namespace.addFeedForwardNamespace("dis 2 coral joint", FeedForwardController.ControlMode.ANGULAR_POSITION);

    public static final double CORAL_JOINT_FORWARD_SPEED = 0.5;
    public static final double CORAL_JOINT_BACKWARD_SPEED = 0.5;

    private static final String NAMESPACE_NAME = "district 2 coral joint";
    private static final double GEAR_RATIO = 1 / 5.0;
    private static final double DEGREES_IN_ROTATIONS = 360;
    private static final double DEGREES_TO_RADIANS = Math.PI / 180;
    private static final double DISTANCE_PER_PULSE = GEAR_RATIO * DEGREES_IN_ROTATIONS * DEGREES_IN_ROTATIONS;

    private final TalonFXWrapper talonFX;
    private final DigitalInput bottomLimit;

    private static District2CoralJoint instance;

    private boolean running;

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
        talonFX.setEncoderConversionFactor(DISTANCE_PER_PULSE / DEGREES_IN_ROTATIONS);
        talonFX.setIdleMode(NeutralModeValue.Brake);
        talonFX.setInverted(true);
        configureDashboard();
    }

    public void brake() {
        talonFX.setIdleMode(NeutralModeValue.Brake);
    }

    private void voltageMove(Voltage voltage) {
        talonFX.set(voltage.in(Units.Volts) / RobotController.getBatteryVoltage());
    }

    public double getPosition() {
        return talonFX.getPosition();
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
        namespace.putRunnable("reset encoder", () -> talonFX.setPosition(0));
        namespace.putCommand("rotate pos", new MoveGenericSubsystem(this, 0.15));
        namespace.putCommand("rotate neg", new MoveGenericSubsystem(this, -0.15));
        Supplier<Double> setpoint = namespace.addConstantDouble("setpoint", 0);
        namespace.putCommand("set voltage", new RunCommand(() -> talonFX.setVoltage(setpoint.get())) {
            @Override
            public void end(boolean True) {
                talonFX.stopMotor();
            }
        });
        namespace.putNumber("voltage", talonFX::get);
        namespace.putCommand("pid", new District2RotateStorage(this, setpoint));
        namespace.putCommand("dont kys", new District2RotateStorage(this, () -> -70.0));
//        namespace.putCommand("pid", new MoveGenericSubsystemWithPID(this, setpoint, talonFX::getPosition,
//                pidSettings, feedForwardSettings));
    }
}
