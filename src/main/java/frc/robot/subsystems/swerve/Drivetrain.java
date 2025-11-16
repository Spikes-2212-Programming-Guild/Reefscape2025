package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.spikes2212.command.DashboardedSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.localization.RobotPoseEstimator;
import frc.robot.util.localization.odometry.OdometryMeasurement;
import frc.robot.util.localization.odometry.PeriodicTaskScheduler;
import frc.robot.util.vision.LimelightService;

import java.util.Arrays;
import java.util.function.Supplier;

// TODO - add an "atRotation" and "atTranslation" methods, and a Field2d, to display on shuffleboard
public class Drivetrain extends DashboardedSubsystem {

    public static final double MAX_SPEED = 4;
    public static final double MIN_SPEED = 0.06;

    /**
     * The frequency the odometry updates at
     */
    public static final int ODOMETRY_FREQUENCY_HZ = 100;

    /**
     * The limit of odometry measurements at a given time, to prevent overflow
     * since it runs on a high frequency
     */
    private static final int ODOMETRY_MEASUREMENT_LIMIT = (ODOMETRY_FREQUENCY_HZ / 50) * 5;

    private static final double ROBOT_WIDTH = 0.6;
    private static final double ROBOT_LENGTH = 0.6;

    private static final Translation2d FRONT_LEFT_WHEEL_DISTANCE_FROM_CENTER =
            new Translation2d(ROBOT_WIDTH / 2, ROBOT_LENGTH / 2);
    private static final Translation2d FRONT_RIGHT_WHEEL_DISTANCE_FROM_CENTER =
            new Translation2d(ROBOT_WIDTH / 2, -ROBOT_LENGTH / 2);
    private static final Translation2d BACK_LEFT_WHEEL_DISTANCE_FROM_CENTER =
            new Translation2d(-ROBOT_WIDTH / 2, ROBOT_LENGTH / 2);
    private static final Translation2d BACK_RIGHT_WHEEL_DISTANCE_FROM_CENTER =
            new Translation2d(-ROBOT_WIDTH / 2, -ROBOT_LENGTH / 2);

    private static final String NAMESPACE_NAME = "drivetrain";

    private final PigeonIMU gyro;
    private final SwerveModule[] swerveModules;
    private final RobotPoseEstimator poseEstimator;
    private final SwerveDriveKinematics kinematics;
    private final LimelightService visionService;

    private final StructArrayPublisher<SwerveModuleState> currentStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("current states", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> desiredStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("desired states", SwerveModuleState.struct).publish();

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain(
                    SwerveModuleHolder.getFrontLeft(), SwerveModuleHolder.getFrontRight(),
                    SwerveModuleHolder.getBackLeft(), SwerveModuleHolder.getBackRight(),
                    new PigeonIMU(-1), PeriodicTaskScheduler.getInstance(),
                    LimelightService.getInstance()
            );
        }
        return instance;
    }

    private Drivetrain(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft,
                       SwerveModule backRight, PigeonIMU gyro, PeriodicTaskScheduler periodicTaskScheduler,
                       LimelightService visionService) {
        super(NAMESPACE_NAME);
        this.swerveModules = new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
        this.gyro = gyro;
        this.visionService = visionService;
        kinematics = new SwerveDriveKinematics(
                FRONT_LEFT_WHEEL_DISTANCE_FROM_CENTER, FRONT_RIGHT_WHEEL_DISTANCE_FROM_CENTER,
                BACK_LEFT_WHEEL_DISTANCE_FROM_CENTER, BACK_RIGHT_WHEEL_DISTANCE_FROM_CENTER
        );
        Supplier<OdometryMeasurement> measurementSupplier = () ->
                new OdometryMeasurement(Timer.getFPGATimestamp(), getHeading(), getModulePositions());
        this.poseEstimator = new RobotPoseEstimator(
                kinematics, getHeading(), getModulePositions(), new Pose2d(),
                measurementSupplier, periodicTaskScheduler, ODOMETRY_FREQUENCY_HZ, ODOMETRY_MEASUREMENT_LIMIT
        );
        configureGyro();
        configureAdvantageKit();
        configureDashboard();
    }

    private void configureAdvantageKit() {
        currentStates.set(new SwerveModuleState[]{
                new SwerveModuleState(), new SwerveModuleState(),
                new SwerveModuleState(), new SwerveModuleState()
        });
        desiredStates.set(new SwerveModuleState[]{
                new SwerveModuleState(), new SwerveModuleState(),
                new SwerveModuleState(), new SwerveModuleState()
        });
    }

    private void configureGyro() {
        gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, ODOMETRY_FREQUENCY_HZ);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("gyro yaw", this::getYaw);
    }

    @Override
    public void periodic() {
        super.periodic();
        poseEstimator.periodic(visionService.captureMeasurement(getYaw(), getRobotRelativeSpeeds()));
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative,
                      boolean usePIDVelocity, double timeStep) {
        ChassisSpeeds speeds;
        if (isFieldRelative) {
            speeds = ChassisSpeeds.discretize(
                    ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getHeading()), timeStep);
        } else {
            speeds = ChassisSpeeds.discretize(new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed), timeStep);
        }
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);
        setTargetModuleStates(states, usePIDVelocity);
        updateAdvantageScope(states);
    }

    private void updateAdvantageScope(SwerveModuleState[] states) {
        currentStates.set(getSwerveModuleStates()); // the states measured by the module sensors
        desiredStates.set(states);
    }

    public Pose2d getPose2d() {
        return poseEstimator.getEstimatedPose();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public double getYaw() {
        return -gyro.getYaw(); // fix pigeon's inverted angle
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        return Arrays.stream(swerveModules)
                .map(SwerveModule::getState)
                .toArray(SwerveModuleState[]::new);
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(swerveModules)
                .map(SwerveModule::getPosition)
                .toArray(SwerveModulePosition[]::new);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(Arrays.stream(swerveModules)
                .map(SwerveModule::getState)
                .toArray(SwerveModuleState[]::new)
        );
    }

    public void resetPose(Pose2d newPose) {
        poseEstimator.resetPose(newPose);
    }

    public void resetGyro() {
        gyro.setYaw(0);
    }

    public void setTargetModuleStates(SwerveModuleState[] targetModuleStates, boolean usePIDVelocity) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].set(targetModuleStates[i], usePIDVelocity);
        }
    }

    public void resetRelativeEncoders() {
        for (SwerveModule module : swerveModules) {
            module.resetRelativeEncoder();
        }
    }

    public void voltageMove(Voltage voltage) {
        for (SwerveModule module : swerveModules) {
            module.sysID(voltage);
        }
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        for (SwerveModule module : swerveModules) {
            module.setIdleMode(neutralMode);
        }
    }

    public void stop() {
        for (SwerveModule module : swerveModules) {
            module.stop();
        }
    }
}
