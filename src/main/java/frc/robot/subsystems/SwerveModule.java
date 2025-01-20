package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule extends DashboardedSubsystem {

    private static final double DRIVE_GEAR_RATIO = 1 / 6.12;
    private static final double TURN_GEAR_RATIO = 1 / 12.8;
    private static final double WHEEL_DIAMETER = 4 * 0.0254;
    private static final double DEGREES_IN_ROTATIONS = 360;
    private static final double SECONDS_IN_MINUTE = 60;
    private static final double ABSOLUTE_POSITION_DISCONTINUITY_POINT = 1;

    private static final double MAX_DISTANCE_TO_ROTATE = 90;
    private static final double DEGREES_TO_FLIP = 180;

    private final TalonFX driveMotor;
    private final SparkMax turnMotor;
    private final CANcoder absoluteEncoder;

    private final boolean cancoderInverted;
    private final boolean driveInverted;
    private final double offset;

    private final RelativeEncoder turnEncoder;

    private final PIDSettings drivePIDSettings;
    private final PIDSettings turnPIDSettings;
    private final FeedForwardSettings driveFeedForwardSettings;
    private final FeedForwardSettings turnFeedForwardSettings;
    private final FeedForwardController turnFeedForwardController;

    private final EncoderConfig turnEncoderConfig;
    private final SparkMaxConfig sparkConfig;
    private final MotorOutputConfigs MotorOutput;

    public SwerveModule(String namespace, TalonFX driveMotor, SparkMax turnMotor, CANcoder absoluteEncoder,
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
        this.turnEncoder = turnMotor.getEncoder();
        this.drivePIDSettings = drivePIDSettings;
        this.turnPIDSettings = turnPIDSettings;
        this.driveFeedForwardSettings = driveFeedForwardSettings;
        this.turnFeedForwardSettings = turnFeedForwardSettings;
        this.turnFeedForwardController = new FeedForwardController(turnFeedForwardSettings,
                FeedForwardController.DEFAULT_PERIOD);
        this.sparkConfig = new SparkMaxConfig();
        MotorOutput = new MotorOutputConfigs();
        turnEncoderConfig = new EncoderConfig();
        configureDriveController();
        configureTurnController();
        configureAbsoluteEncoder();
    }

    public void configureDriveController() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        //@TODO check if this is correct
        config.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO * WHEEL_DIAMETER * Math.PI;
        config.Feedback.RotorToSensorRatio = (DRIVE_GEAR_RATIO * WHEEL_DIAMETER * Math.PI) / SECONDS_IN_MINUTE;
        driveMotor.getConfigurator().apply(config);
        Slot0Configs driveConfigs = new Slot0Configs();
        driveConfigs.kP = drivePIDSettings.getkP();
        driveConfigs.kI = drivePIDSettings.getkI();
        driveConfigs.kD = drivePIDSettings.getkD();
        driveConfigs.kS = driveFeedForwardSettings.getkS();
        driveConfigs.kV = driveFeedForwardSettings.getkV();
        driveConfigs.kA = driveFeedForwardSettings.getkA();
        driveMotor.getConfigurator().apply(driveConfigs);
        driveMotor.getConfigurator().apply(MotorOutput.withInverted(driveInverted ?
                InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive));
    }

    public void configureTurnController() {
        turnEncoderConfig.positionConversionFactor(DRIVE_GEAR_RATIO * DEGREES_IN_ROTATIONS);
        turnEncoderConfig.velocityConversionFactor((TURN_GEAR_RATIO * DEGREES_IN_ROTATIONS) / SECONDS_IN_MINUTE);
        ClosedLoopConfig turnClosedLoopConfig = new ClosedLoopConfig();
        turnClosedLoopConfig.pid(turnPIDSettings.getkP(), turnPIDSettings.getkI(), turnPIDSettings.getkD());
        turnEncoderConfig.inverted(cancoderInverted);
        sparkConfig.apply(turnClosedLoopConfig);
        turnMotor.configure(sparkConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    public void configureAbsoluteEncoder() {
        MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(ABSOLUTE_POSITION_DISCONTINUITY_POINT)
                .withSensorDirection(cancoderInverted ? SensorDirectionValue.Clockwise_Positive :
                        SensorDirectionValue.CounterClockwise_Positive).withMagnetOffset(offset);
        absoluteEncoder.getConfigurator().apply(magnetConfigs);
    }

    private void turnConfigureFF() {
        turnFeedForwardController.setGains(turnFeedForwardSettings);
    }

    private void setSpeed(double speed, boolean usePID) {
        if (usePID) {
            configureDriveController();
            driveMotor.setControl(new VelocityDutyCycle(speed));
        } else driveMotor.set(speed / Drivetrain.MAX_SPEED);
    }

    private void setAngle(double angle) {
        configureTurnController();
        turnConfigureFF();
        double feedForward = turnFeedForwardController.calculate(angle);
        turnMotor.getClosedLoopController().setReference(angle, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0,
                feedForward, SparkClosedLoopController.ArbFFUnits.kPercentOut);
    }

    public void stop() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }

    public void set(SwerveModuleState state, boolean usePID) {
        if (state.speedMetersPerSecond < Drivetrain.MIN_SPEED){
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
        while (Math.abs(desiredAngle - currentAngle) > MAX_DISTANCE_TO_ROTATE) {
            if (desiredAngle - currentAngle > 0) {
                desiredAngle -= DEGREES_TO_FLIP;
            }
            else {
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
        turnEncoder.setPosition(getAbsoluteAngle());
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("absolute angle", this::getAbsoluteAngle);
        namespace.putNumber("relative angle", turnEncoder::getPosition);
    }
}
