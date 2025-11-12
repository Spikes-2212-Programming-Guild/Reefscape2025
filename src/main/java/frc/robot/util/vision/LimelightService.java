package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.localization.vision.StandardDeviations;
import frc.robot.util.localization.vision.VisionMeasurement;

public class LimelightService {

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

    private static LimelightService instance;

    public static LimelightService getInstance() {
        return instance != null ? instance : (instance = new LimelightService());
    }

    private LimelightService() {

    }

    /**
     * Fetches and processes the latest Limelight pose estimate.
     *
     * @param yaw    current robot yaw (degrees)
     * @param speeds current robot chassis speeds
     * @return a {@link VisionMeasurement} if valid; otherwise {@code null}
     */
    public VisionMeasurement captureMeasurement(double yaw, ChassisSpeeds speeds) {
        LimelightHelpers.SetRobotOrientation(
                LIMELIGHT_NAME, yaw, 0, 0, 0, 0, 0
        );

        var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
        if (estimate == null || estimate.pose == null ||
                estimate.tagCount < 1 || estimate.tagSpan < MIN_TAG_SPAN) return null;

        if (!isReliable(estimate.avgTagDist, speeds)) return null;

        return createMeasurement(
                estimate.timestampSeconds, estimate.avgTagDist, estimate.tagCount, estimate.pose
        );
    }

    /**
     * Determines whether a vision measurement is reliable enough to use based on robot motion
     * and average tag distance.
     *
     * @param averageTagDistance the average distance to the visible tag(s).
     * @param speeds             the current robot speeds
     * @return if the measurement should be trusted
     */
    private boolean isReliable(double averageTagDistance, ChassisSpeeds speeds) {
        if (averageTagDistance >= MAX_DISTANCE) return false;
        double driveVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double turnVelocity = Math.abs(speeds.omegaRadiansPerSecond);
        return driveVelocity <= MAX_DRIVE_SPEED && turnVelocity <= MAX_TURN_SPEED;
    }

    /**
     * Creates a {@link VisionMeasurement} containing the estimated robot pose
     * and computed standard deviations based on vision data quality.
     *
     * @param timestamp the time (in seconds) when the measurement was captured
     * @param distance  the average distance to the visible AprilTag(s)
     * @param tagCount  the number of AprilTag(s) contributing to the estimate
     * @param pose      the estimated robot {@link Pose2d} from vision
     * @return a constructed {@link VisionMeasurement} with associated uncertainty
     */
    private VisionMeasurement createMeasurement(double timestamp, double distance, int tagCount, Pose2d pose) {
        double transStd = calculateStandardDeviation(STD_DEV_DRIVE_EXPONENT, distance, tagCount);
        double rotStd = calculateStandardDeviation(STD_DEV_ROTATION_EXPONENT, distance, tagCount);
        return new VisionMeasurement(new StandardDeviations(transStd, rotStd), pose, timestamp);
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
    private double calculateStandardDeviation(double exponent, double distance, int tagCount) {
        return exponent * (distance * distance) / tagCount;
    }
}
