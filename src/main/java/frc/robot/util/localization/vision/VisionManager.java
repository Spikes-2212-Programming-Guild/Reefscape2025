package frc.robot.util.localization.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Handles vision-based pose estimation using Limelight.
 * <p>
 * This class integrates Limelight vision data into the {@link SwerveDrivePoseEstimator},
 * automatically filtering unreliable measurements based on distance and robot motion.
 * </p>
 *
 * <h1>Limelight Setup Requirements:</h1>
 * <ol>
 *     <li>Upload a field map to the Limelight.</li>
 *     <li>Configure <b>Pipeline 0</b> as an AprilTag pipeline with 3D functionality enabled.</li>
 *     <li>Set the Limelight's physical pose relative to the robot center in the Limelight UI.</li>
 *     <li>Ensure the Limelight is running the "MegaTag2" algorithm for multi-tag tracking.</li>
 * </ol>
 *
 * @author Itay Zadok
 */
public class VisionManager {

    /**
     * The name of the Limelight as configured in the NetworkTables.
     */
    private static final String LIMELIGHT_NAME = "limelight";

    /**
     * Thresholds for measurement reliability.
     */
    private static final double MAX_DRIVE_SPEED = -1; // (m / s)
    private static final double MAX_TURN_SPEED = -1; // (rad / s)
    private static final double MAX_DISTANCE = -1; // (m)
    private static final double MIN_TAG_SPAN = -1; // (s)

    /**
     * Calibrated gains for standard deviation scaling.
     */
    private static final double STD_DEV_DRIVE_EXPONENT = -1;
    private static final double STD_DEV_ROTATION_EXPONENT = -1;

    private static final StandardDeviations BASE_STANDARD_DEVIATIONS =
            new StandardDeviations(-1, -1);

    private final SwerveDrivePoseEstimator poseEstimator;

    /**
     * Constructs a VisionManager linked to a given {@link SwerveDrivePoseEstimator}.
     *
     * @param poseEstimator the estimator to update with vision data
     */
    public VisionManager(SwerveDrivePoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
        poseEstimator.setVisionMeasurementStdDevs(BASE_STANDARD_DEVIATIONS.toMatrix());
    }

    /**
     * Should be called periodically (e.g., in {@code Drivetrain.periodic()}).
     * Updates the pose estimator with new Limelight vision data if reliable.
     *
     * @param gyroYaw    the robot’s current yaw angle
     * @param robotSpeed the current chassis speed (used to reject poor measurements)
     */
    public void periodic(double gyroYaw, ChassisSpeeds robotSpeed) {
        LimelightHelpers.SetRobotOrientation(LIMELIGHT_NAME, gyroYaw,
                0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.PoseEstimate measurement =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

        if (measurement == null || measurement.tagCount < 1 || measurement.tagSpan < MIN_TAG_SPAN) return;

        double avgTagDistance = measurement.avgTagDist;
        if (!isMeasurementReliable(avgTagDistance, robotSpeed)) return;

        addVisionMeasurement(
                measurement.timestampSeconds,
                avgTagDistance,
                measurement.tagCount,
                measurement.pose
        );
    }

    /**
     * Adds a vision-based measurement to the pose estimator, applying dynamically
     * scaled standard deviations based on tag count and distance.
     *
     * @param timestamp      the time when the measurement was taken
     * @param distance       the average distance to visible tag(s)
     * @param tagCount       the number of tag(s) contributing to the estimate
     * @param robotFieldPose the robot's estimated pose on the field
     */
    private void addVisionMeasurement(double timestamp, double distance, int tagCount, Pose2d robotFieldPose) {
        double translationStdDev = calculateStandardDeviation(STD_DEV_DRIVE_EXPONENT, distance, tagCount);
        double rotationStdDev = calculateStandardDeviation(STD_DEV_ROTATION_EXPONENT, distance, tagCount);

        StandardDeviations standardDeviations = new StandardDeviations(translationStdDev, rotationStdDev);
        poseEstimator.addVisionMeasurement(robotFieldPose, timestamp, standardDeviations.toMatrix());
    }

    /**
     * Determines whether a vision measurement is reliable based on robot motion
     * and average tag distance.
     *
     * @param avgDistance the average distance to the visible tag(s).
     * @param speeds      the current robot speeds
     * @return if the measurement should be trusted
     */
    private boolean isMeasurementReliable(double avgDistance, ChassisSpeeds speeds) {
        if (avgDistance >= MAX_DISTANCE) return false;

        double driveVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double turnVelocity = Math.abs(speeds.omegaRadiansPerSecond);

        return driveVelocity <= MAX_DRIVE_SPEED &&
                turnVelocity <= MAX_TURN_SPEED;
    }

    /**
     * Computes the standard deviation used for vision measurements.
     * <p>
     * As distance increases or tag count decreases, the result becomes less
     * confident (larger standard deviation).
     * </p>
     *
     * @param exponent a calibrated gain
     * @param distance the average distance to the visible tag(s).
     * @param tagCount the number of visible tag(s)
     * @return the computed standard deviation
     */
    public static double calculateStandardDeviation(double exponent, double distance, int tagCount) {
        return exponent * (distance * distance) / tagCount;
    }
}
