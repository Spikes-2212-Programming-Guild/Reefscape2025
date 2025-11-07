package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.spikes2212.command.DashboardedSubsystem;
import com.studica.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.localization.RobotPoseEstimator;
import frc.robot.util.localization.odometry.OdometrySource;
import frc.robot.util.localization.odometry.PeriodicTaskScheduler;

import java.util.Arrays;

// TODO - add an "atRotation" and "atTranslation" methods, and a Field2d, to display on shuffleboard
public class Drivetrain extends DashboardedSubsystem implements OdometrySource {

    public static final double MAX_SPEED = 4;
    public static final double MIN_SPEED = 0.06;

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

    private final AHRS gyro;
    private final SwerveModule[] swerveModules;
    private final RobotPoseEstimator poseEstimator;
    private final SwerveDriveKinematics kinematics;

    private SysIdRoutine sysIdRoutine;
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
                    new AHRS(AHRS.NavXComType.kMXP_SPI), PeriodicTaskScheduler.getInstance());
        }
        return instance;
    }

    private Drivetrain(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft,
                       SwerveModule backRight, AHRS gyro, PeriodicTaskScheduler periodicTaskScheduler) {
        super(NAMESPACE_NAME);
        this.swerveModules = new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
        this.gyro = gyro;
        kinematics = new SwerveDriveKinematics(
                FRONT_LEFT_WHEEL_DISTANCE_FROM_CENTER, FRONT_RIGHT_WHEEL_DISTANCE_FROM_CENTER,
                BACK_LEFT_WHEEL_DISTANCE_FROM_CENTER, BACK_RIGHT_WHEEL_DISTANCE_FROM_CENTER
        );
        this.poseEstimator = new RobotPoseEstimator(
                new SwerveDrivePoseEstimator(kinematics, getHeading(), getSwerveModulePositions(), new Pose2d()),
                this, periodicTaskScheduler
        );
        configureSysId();
        configureAdvantageKit();
        configureDashboard();
    }

    private void configureSysId() {
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, null, null,
                        state -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(this::voltageMove, null, this)
        );
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

    @Override
    public void configureDashboard() {
        namespace.putNumber("gyro yaw", this::getYaw);
        namespace.putCommand("quasistatic forward", sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        namespace.putCommand("quasistatic reverse", sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
        namespace.putCommand("dynamic forward", sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
        namespace.putCommand("dynamic reverse", sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    @Override
    public void periodic() {
        super.periodic();
        poseEstimator.periodic(getYaw(), getRobotRelativeSpeeds());
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative,
                      double timeStep, boolean usePIDVelocity) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                getChassisSpeeds(isFieldRelative, xSpeed, ySpeed, rotationSpeed, timeStep)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);
        setTargetModuleStates(states, usePIDVelocity);
        updateAdvantageScope(states);
    }

    private void updateAdvantageScope(SwerveModuleState[] states) {
        currentStates.set(getSwerveModuleStates()); // the states measured by the sensors
        desiredStates.set(states);
    }

    public void stop() {
        for (SwerveModule module : swerveModules) {
            module.stop();
        }
    }

    private ChassisSpeeds getChassisSpeeds(boolean fieldRelative, double xSpeed, double ySpeed,
                                           double rotationSpeed, double timeStep) {
        if (fieldRelative) {
            return ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed,
                    getHeading()), timeStep);
        } else {
            return ChassisSpeeds.discretize(new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed), timeStep);
        }
    }

    public Pose2d getPose2d() {
        return poseEstimator.getEstimatedPose();
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public double getYaw() {
        return -gyro.getAngle(); // fix navX's inverted angle
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        return Arrays.stream(swerveModules)
                .map(SwerveModule::getState)
                .toArray(SwerveModuleState[]::new);
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
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
        gyro.reset();
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

    @Override
    public double getTimestamp() {
        return gyro.getLastSensorTimestamp();
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[0];
    }
}
