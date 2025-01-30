package frc.robot.subsystems;

import com.spikes2212.command.DashboardedSubsystem;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Drivetrain extends DashboardedSubsystem {

    public static final double MAX_SPEED = 4;
    public static final double MIN_SPEED = 0.05;
    public static final double MAX_TURN_SPEED = 3;

    private static final double TRACK_WIDTH = 60;
    private static final double TRACK_LENGTH = 60;

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

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain(SwerveModuleHolder.getFrontLeft(), SwerveModuleHolder.getFrontRight(),
                    SwerveModuleHolder.getBackLeft(), SwerveModuleHolder.getBackRight(),
                    new AHRS(AHRS.NavXComType.kMXP_SPI));
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
        this.gyro = gyro;
        kinematics = new SwerveDriveKinematics(FRONT_LEFT_WHEEL_POSITION,
                FRONT_RIGHT_WHEEL_POSITION, BACK_LEFT_WHEEL_POSITION, BACK_RIGHT_WHEEL_POSITION);
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

    public void resetRelativeEncoders() {
        frontLeft.resetRelativeEncoder();
        frontRight.resetRelativeEncoder();
        backLeft.resetRelativeEncoder();
        backRight.resetRelativeEncoder();
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("gyro yaw", gyro::getAngle);
    }
}
