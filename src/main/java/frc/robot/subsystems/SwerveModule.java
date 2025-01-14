package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule extends DashboardedSubsystem {

    public static final double MAX_SPEED = 4;
    public static final double MIN_SPEED = 0.4;
    public static final double MAX_TURN = 3;
    private static final double DRIVE_GEAR_RATIO = 1 / 6.12;
    private static final double TURN_GEAR_RATIO = 1 / 12.8;
    private static final double WHEEL_DIAMETER = 4 * 0.0254;
    private static final double DEGREES_IN_ROTATIONS = 360;
    private static final double SECONDS_IN_MINUTE = 60;
    private static final double ABSOLUTE_POSITION = 1;

    private final TalonFX driveMotor;
    private final SparkMax turnMotor;
    private final CANcoder absoluteEncoder;
    private final boolean cancoderInverted;
    private final boolean driveInverted;
    private final double offset;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    private final PIDSettings drivePIDSettings;
    private final PIDSettings turnPIDSettings;
    private final FeedForwardSettings driveFeedForwardSettings;
    private final FeedForwardSettings turnFeedForwardSettings;
    private final EncoderConfig driveEncoderConfig;
    private final EncoderConfig turnEncoderConfig;

    public SwerveModule(String namespace, TalonFX driveMotor, SparkMax turnMotor, CANcoder absoluteEncoder,
                        boolean cancoderInverted, boolean driveInverted, double offset,
                        RelativeEncoder driveEncoder, RelativeEncoder turnEncoder,
                        PIDSettings drivePIDSettings, PIDSettings turnPIDSettings,
                        FeedForwardSettings driveFeedForwardSettings,
                        FeedForwardSettings turnFeedForwardSettings) {
        super(namespace);
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.absoluteEncoder = absoluteEncoder;
        this.cancoderInverted = cancoderInverted;
        this.driveInverted = driveInverted;
        this.offset = offset;
        this.driveEncoder = driveEncoder;
        this.turnEncoder = turnEncoder;
        this.drivePIDSettings = drivePIDSettings;
        this.turnPIDSettings = turnPIDSettings;
        this.driveFeedForwardSettings = driveFeedForwardSettings;
        this.turnFeedForwardSettings = turnFeedForwardSettings;
        turnEncoderConfig = new EncoderConfig();
        driveEncoderConfig = new EncoderConfig();
        createDriveConfig();
        createTurnController();
    }

    private void createDriveConfig() {
        driveEncoderConfig.positionConversionFactor(DRIVE_GEAR_RATIO * WHEEL_DIAMETER * Math.PI);
        driveEncoderConfig.velocityConversionFactor((DRIVE_GEAR_RATIO * WHEEL_DIAMETER * Math.PI) / SECONDS_IN_MINUTE);
    }


    public void configureDriveController() {
        Slot0Configs driveConfigs = new Slot0Configs();
        driveConfigs.kP = drivePIDSettings.getkP();
        driveConfigs.kI = drivePIDSettings.getkI();
        driveConfigs.kD = drivePIDSettings.getkD();
        driveConfigs.kS = driveFeedForwardSettings.getkS();
        driveConfigs.kV = driveFeedForwardSettings.getkV();
        driveConfigs.kA = driveFeedForwardSettings.getkA();
        driveMotor.getConfigurator().apply(driveConfigs);
        driveEncoderConfig.inverted(driveInverted);
    }

    private void createTurnController() {
        turnEncoderConfig.positionConversionFactor(DRIVE_GEAR_RATIO * DEGREES_IN_ROTATIONS);
        turnEncoderConfig.velocityConversionFactor((TURN_GEAR_RATIO * DEGREES_IN_ROTATIONS) / SECONDS_IN_MINUTE);
    }

    public void configureTurnController() {
        ClosedLoopConfig turnClosedLoopConfig = new ClosedLoopConfig();
        turnClosedLoopConfig.pid(turnPIDSettings.getkP(), turnPIDSettings.getkI(), turnPIDSettings.getkD());
        turnEncoderConfig.inverted(cancoderInverted);
    }

    public void configureAbsoluteEncoder() {
        MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(ABSOLUTE_POSITION)
                .withSensorDirection(cancoderInverted ? SensorDirectionValue.Clockwise_Positive :
                        SensorDirectionValue.CounterClockwise_Positive).withMagnetOffset(offset);
        absoluteEncoder.getConfigurator().apply(magnetConfigs);
    }

    private void setSpeed(double speed, boolean usePID) {
        if (usePID) {
            configureDriveController();
            driveMotor.setControl(new VelocityDutyCycle(speed));
        } else driveMotor.set(speed / MAX_SPEED);
    }

    private void setAngle(double angle) {
        configureTurnController();
        turnMotor.getClosedLoopController().setReference(angle, SparkBase.ControlType.kVelocity);
    }

    public void stop() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }

    public void set(SwerveModuleState state, boolean usePID) {
        if (state.speedMetersPerSecond < MIN_SPEED){
            stop();
            return;
        }
        state = optimize(state, turnEncoder.getPosition());
        setAngle(state.angle.getDegrees());
        setSpeed(state.speedMetersPerSecond, usePID);
    }

    private double normalizeAngleRelativeToEncoder(double currentAngle, double desiredAngle) {
        int rotations = (int) (currentAngle / DEGREES_IN_ROTATIONS);
        if (currentAngle < 0) rotations--;
        desiredAngle += rotations * DEGREES_IN_ROTATIONS;
        return desiredAngle;
    }

    private SwerveModuleState optimize(SwerveModuleState state, double currentAngle) {
        double desiredAngle = normalizeAngleRelativeToEncoder(currentAngle, state.angle.getDegrees());
        while (Math.abs(desiredAngle - currentAngle) > 90) {
            if (desiredAngle - currentAngle > 0) {
                desiredAngle -= 180;
            }
            else {
                desiredAngle += 180;
            }
            state.speedMetersPerSecond *= -1;
        }
        return new SwerveModuleState(state.speedMetersPerSecond, Rotation2d.fromDegrees(desiredAngle));
    }

    public double getAbsoluteAngle() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * DEGREES_IN_ROTATIONS;
    }

    public void resetRelativeEncoder() {
        turnEncoder.setPosition(getAbsoluteAngle());
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("absolute angle", this::getAbsoluteAngle);
        namespace.putNumber("velocity", driveEncoder::getVelocity);
        namespace.putNumber("relative angle", turnEncoder::getPosition);
    }
}
