package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.UnifiedControlMode;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule extends DashboardedSubsystem {

    private static final double DRIVE_GEAR_RATIO = 1 / 6.12;
    private static final double TURN_GEAR_RATIO = 1 / 12.8;
    private static final double WHEEL_DIAMETER_INCHES = 4;
    private static final double INCHES_TO_METERS = 0.0254;
    private static final double DEGREES_IN_ROTATIONS = 360;
    private static final double SECONDS_IN_MINUTE = 60;

    private static final double MAX_DISTANCE_TO_ROTATE = 90;
    private static final double DEGREES_TO_FLIP = 180;
    private static final double ABSOLUTE_POSITION_DISCONTINUITY_POINT = 1;

    private final TalonFXWrapper driveMotor;
    private final SparkWrapper turnMotor;
    private final CANcoder absoluteEncoder;

    private final boolean cancoderInverted;
    private final boolean driveInverted;
    private final double offset;

    private final PIDSettings drivePIDSettings;
    private final PIDSettings turnPIDSettings;
    private final FeedForwardSettings driveFeedForwardSettings;
    private final FeedForwardSettings turnFeedForwardSettings;

    public SwerveModule(String namespace, TalonFXWrapper driveMotor, SparkWrapper turnMotor, CANcoder absoluteEncoder,
                        boolean cancoderInverted, boolean driveInverted, double offset,
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
        this.drivePIDSettings = drivePIDSettings;
        this.turnPIDSettings = turnPIDSettings;
        this.driveFeedForwardSettings = driveFeedForwardSettings;
        this.turnFeedForwardSettings = turnFeedForwardSettings;
        configureDriveController();
        configureTurnController();
        configureAbsoluteEncoder();
        configureDashboard();
    }

    public void configureDriveController() {
        //@TODO check if this is correct
        driveMotor.setEncoderConversionFactor(DRIVE_GEAR_RATIO * WHEEL_DIAMETER_INCHES
                * INCHES_TO_METERS * Math.PI);
        driveMotor.setInverted(driveInverted);
    }

    public void configureTurnController() {
        turnMotor.setPositionConversionFactor(TURN_GEAR_RATIO * DEGREES_IN_ROTATIONS);
        turnMotor.setVelocityConversionFactor((TURN_GEAR_RATIO * DEGREES_IN_ROTATIONS) / SECONDS_IN_MINUTE);
        turnMotor.setInverted(cancoderInverted);
    }

    public void configureAbsoluteEncoder() {
        MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(ABSOLUTE_POSITION_DISCONTINUITY_POINT)
                .withSensorDirection(cancoderInverted ? SensorDirectionValue.Clockwise_Positive :
                        SensorDirectionValue.CounterClockwise_Positive).withMagnetOffset(offset);
        absoluteEncoder.getConfigurator().apply(magnetConfigs);
    }

    private void setSpeed(double speed, boolean usePID) {
        if (usePID) {
            driveMotor.pidSet(UnifiedControlMode.VELOCITY, speed, drivePIDSettings, driveFeedForwardSettings,
                    true);
        } else driveMotor.set(speed / Drivetrain.MAX_SPEED);
    }

    private void setAngle(double angle) {
        turnMotor.pidSet(UnifiedControlMode.POSITION, angle, turnPIDSettings, turnFeedForwardSettings,
                true);
    }

    public void stop() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }

    public void set(SwerveModuleState state, boolean usePID) {
        if (Math.abs(state.speedMetersPerSecond) < Drivetrain.MIN_SPEED) {
            stop();
            return;
        }
        state = optimize(state, turnMotor.getPosition());
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
        while (Math.abs(desiredAngle - currentAngle) > MAX_DISTANCE_TO_ROTATE) {
            if (desiredAngle - currentAngle > 0) {
                desiredAngle -= DEGREES_TO_FLIP;
            } else {
                desiredAngle += DEGREES_TO_FLIP;
            }
            state.speedMetersPerSecond *= -1;
        }
        return new SwerveModuleState(state.speedMetersPerSecond, Rotation2d.fromDegrees(desiredAngle));
    }

    public double getAbsoluteAngle() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * DEGREES_IN_ROTATIONS;
    }

    public void resetRelativeEncoder() {
        turnMotor.setPosition(getAbsoluteAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition(),
                Rotation2d.fromDegrees(getAbsoluteAngle()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity(),
                Rotation2d.fromDegrees(getAbsoluteAngle()));
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("absolute angle", this::getAbsoluteAngle);
        namespace.putNumber("relative angle", turnMotor::getPosition);
    }
}
