package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.spikes2212.command.DashboardedSubsystem;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;

public class Drivetrain extends DashboardedSubsystem {

    public static final double MAX_SPEED = 4;
    public static final double MIN_SPEED = 0.4;
    public static final double MAX_TURN_SPEED = 3;

    private static final double TRACK_WIDTH = -1;
    private static final double TRACK_LENGTH = -1;

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

    private SwerveModulePosition[] stupidPointlessArray;
    private final SwerveDriveOdometry odometry;
    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain(SwerveModuleHolder.getFrontLeft(), SwerveModuleHolder.getFrontRight(),
                    SwerveModuleHolder.getBackLeft(), SwerveModuleHolder.getBackRight(),
                    new AHRS(AHRS.NavXComType.kI2C));
        }
        return instance;
    }

    private Drivetrain(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft,
                       SwerveModule backRight, AHRS gyro) {
        super(NAMESPACE_NAME);
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        stupidPointlessArray = new SwerveModulePosition[] {frontLeft.getPosition(),
                frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
        kinematics = new SwerveDriveKinematics(FRONT_LEFT_WHEEL_POSITION,
                FRONT_RIGHT_WHEEL_POSITION, BACK_LEFT_WHEEL_POSITION, BACK_RIGHT_WHEEL_POSITION);
        this.gyro = gyro;
        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), stupidPointlessArray,
                new Pose2d());
        RobotConfig config;
        try{
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
                        new PIDConstants(0, 0.0, 0.0), // Translation PID constants
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
    }

    @Override
    public void periodic() {
        super.periodic();
        stupidPointlessArray = new SwerveModulePosition[]{frontLeft.getPosition(),
                frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
        odometry.update(gyro.getRotation2d(), stupidPointlessArray);
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative,
                      boolean usePID) {
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed,
                    gyro.getRotation2d());
        }
        else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
        }
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds, CENTER_OF_ROBOT);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);
        frontLeft.set(states[0], usePID);
        frontRight.set(states[1], usePID);
        backLeft.set(states[2], usePID);
        backRight.set(states[3], usePID);
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void resetRelativeEncoder() {
        frontLeft.resetRelativeEncoder();
        frontRight.resetRelativeEncoder();
        backLeft.resetRelativeEncoder();
        backRight.resetRelativeEncoder();
    }

    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
    }

    public void resetPose2d(Pose2d desiredPose) {
        odometry.resetPosition(gyro.getRotation2d(), stupidPointlessArray, desiredPose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(),
                backLeft.getState(), backRight.getState());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond,
                false, true);
    }

    @Override
    public void configureDashboard() {
    }
}
