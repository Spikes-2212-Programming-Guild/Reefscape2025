package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.studica.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Drive;
import frc.robot.util.VisionService;

public class Drivetrain extends DashboardedSubsystem {

    public static final double MAX_SPEED = 4;
    public static final double MIN_SPEED = 0.06;
    public static final double MAX_TURN_SPEED = 3;

    private static final double TRACK_WIDTH = 0.6;
    private static final double TRACK_LENGTH = 0.6;

    private static final Translation2d CENTER_OF_ROBOT = new Translation2d(0, 0);
    private static final Translation2d FRONT_LEFT_WHEEL_POSITION =
            new Translation2d(TRACK_WIDTH / 2, TRACK_LENGTH / 2);
    private static final Translation2d FRONT_RIGHT_WHEEL_POSITION =
            new Translation2d(TRACK_WIDTH / 2, -TRACK_LENGTH / 2);
    private static final Translation2d BACK_LEFT_WHEEL_POSITION =
            new Translation2d(-TRACK_WIDTH / 2, TRACK_LENGTH / 2);
    private static final Translation2d BACK_RIGHT_WHEEL_POSITION =
            new Translation2d(-TRACK_WIDTH / 2, -TRACK_LENGTH / 2);

    private static final String NAMESPACE_NAME = "drivetrain";

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final AHRS gyro;
    private final SwerveDriveKinematics kinematics;

    private final SwerveDriveOdometry odometry;
    private final VisionService visionService;
    private final SysIdRoutine sysIdRoutine;
    private final StructArrayPublisher<SwerveModuleState> currentStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("current states", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> desiredStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("desired states", SwerveModuleState.struct).publish();

    private SwerveModulePosition[] swerveModulePositions;

    private Pose2d currentPose;

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain(SwerveModuleHolder.getFrontLeft(), SwerveModuleHolder.getFrontRight(),
                    SwerveModuleHolder.getBackLeft(), SwerveModuleHolder.getBackRight(),
                    new AHRS(AHRS.NavXComType.kMXP_SPI), VisionService.getInstance());
        }
        return instance;
    }

    private Drivetrain(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft,
                       SwerveModule backRight, AHRS gyro, VisionService visionService) {
        super(NAMESPACE_NAME);
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.gyro = gyro;
        this.visionService = visionService;
        swerveModulePositions = new SwerveModulePosition[]{frontLeft.getPosition(),
                frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
        kinematics = new SwerveDriveKinematics(FRONT_LEFT_WHEEL_POSITION,
                FRONT_RIGHT_WHEEL_POSITION, BACK_LEFT_WHEEL_POSITION, BACK_RIGHT_WHEEL_POSITION);
        odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), swerveModulePositions,
                new Pose2d());
        currentPose = new Pose2d();
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            config = null;
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose2d, // Robot pose supplier
                this::resetPose2d, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(0.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(0.01, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        currentStates.set(
                new SwerveModuleState[]{
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState()
                }
        );
        desiredStates.set(
                new SwerveModuleState[]{
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState()
                }
        );
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, null, null, state -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(this::sysID, null, this)
        );
        configureDashboard();
    }

    @Override
    public void periodic() {
        super.periodic();
        swerveModulePositions = new SwerveModulePosition[]{frontLeft.getPosition(),
                frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
        if (visionService.hasTarget()) {
            if (visionService.getTargetRelativePose() != null) {
                odometry.resetPosition(getRotation2d(), swerveModulePositions,
                        visionService.getTargetRelativePose());
            }
        }
        currentPose = odometry.update(getRotation2d(), new SwerveModulePosition[] {
                frontLeft.getPosition(), frontRight.getPosition(),
                backLeft.getPosition(), backRight.getPosition()
        });
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative,
                      boolean usePID, double timestep) {
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed,
                    getRotation2d());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
        }
        speeds = ChassisSpeeds.discretize(speeds, timestep);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds, CENTER_OF_ROBOT);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);
        frontLeft.set(states[0], usePID);
        frontRight.set(states[1], usePID);
        backLeft.set(states[2], usePID);
        backRight.set(states[3], usePID);
        currentStates.set(
                new SwerveModuleState[]{
                        frontLeft.getState(),
                        frontRight.getState(),
                        backLeft.getState(),
                        backRight.getState()
                }
        );
        desiredStates.set(states);
    }

    private final PIDController turnPIDController = new PIDController(0, 0, 0);
    private final FeedForwardController turnFeedForwardController = new FeedForwardController(
            new FeedForwardSettings(0, 0, 0, FeedForwardController.ControlMode.LINEAR_POSITION)
    );
    public void sysID(Voltage voltage) {
        drive((voltage.in(Units.Volts) / RobotController.getBatteryVoltage()) * MAX_SPEED, 0,
                turnPIDController.calculate(getYaw(), 0) + turnFeedForwardController.calculate(getYaw(), 0),
                true, false, 0.02);
    }

    public void sysIDNoAngle(Voltage voltage) {
        drive((voltage.in(Units.Volts) / RobotController.getBatteryVoltage()) * MAX_SPEED, 0, 0, true, false, 0.02);
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void resetRelativeEncoders() {
        frontLeft.resetRelativeEncoder();
        frontRight.resetRelativeEncoder();
        backLeft.resetRelativeEncoder();
        backRight.resetRelativeEncoder();
    }

    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
    }

    public void resetPose2d(Pose2d desiredPose) {
        odometry.resetPosition(getRotation2d(), swerveModulePositions, desiredPose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(),
                backLeft.getState(), backRight.getState());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond,
                false, true, 0.02);
    }
      
    public void resetGyro() {
        gyro.reset();
    }

    public double getYaw() {
        return -gyro.getAngle();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        frontLeft.setIdleMode(neutralMode);
        frontRight.setIdleMode(neutralMode);
        backLeft.setIdleMode(neutralMode);
        backRight.setIdleMode(neutralMode);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("gyro yaw", this::getYaw);
        namespace.putCommand("check skew", new Drive(this, () -> 0.0, () -> 0.5, () -> 1.0, true, false, false));
        namespace.putCommand("quasistatic forward", sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        namespace.putCommand("quasistatic reverse", sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
        namespace.putCommand("dynamic forward", sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
        namespace.putCommand("dynamic reverse", sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
        namespace.putCommand("set angle", new InstantCommand(() -> gyro.setAngleAdjustment(0)));
    }
}
