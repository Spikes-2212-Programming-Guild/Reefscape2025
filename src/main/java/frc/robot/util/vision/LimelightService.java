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
        if (instance == null) {
            instance = new LimelightService();
        }
        return instance;
    }

    private LimelightService() {

    }

    /**
     * @param yaw    the current gyro yaw
     * @param speeds the current robot speed
     * @return the latest vision measurement {@link VisionMeasurement}, can be null
     */
    public VisionMeasurement getVisionMeasurement(double yaw, ChassisSpeeds speeds) {
        LimelightHelpers.SetRobotOrientation(LIMELIGHT_NAME, yaw,
                0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.PoseEstimate measurement =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

        if (measurement == null || measurement.tagCount < 1 || measurement.tagSpan < MIN_TAG_SPAN) return null;

        double avgTagDistance = measurement.avgTagDist;
        if (!isMeasurementReliable(avgTagDistance, speeds)) return null;

        return buildMeasurement(
                measurement.timestampSeconds, measurement.avgTagDist, measurement.tagCount, measurement.pose
        );
    }

    /**
     * Builds a {@link VisionMeasurement} object made of an estimated robot pose and {@link StandardDeviations}
     *
     * @param timestamp     the time when the measurement was taken
     * @param distance      the average distance to visible tag(s)
     * @param tagCount      the number of tag(s) contributing to the estimate
     * @param estimatedPose the estimated robot pose
     */
    private VisionMeasurement buildMeasurement(double timestamp, double distance, int tagCount, Pose2d estimatedPose) {
        double translationStdDev = calculateStandardDeviation(STD_DEV_DRIVE_EXPONENT, distance, tagCount);
        double rotationStdDev = calculateStandardDeviation(STD_DEV_ROTATION_EXPONENT, distance, tagCount);
        StandardDeviations standardDeviations = new StandardDeviations(translationStdDev, rotationStdDev);
        return new VisionMeasurement(standardDeviations, estimatedPose, timestamp);
    }

    /**
     * Determines whether a vision measurement is reliable enough to use based on robot motion
     * and average tag distance.
     *
     * @param averageTagDistance the average distance to the visible tag(s).
     * @param speeds             the current robot speeds
     * @return if the measurement should be trusted
     */
    private boolean isMeasurementReliable(double averageTagDistance, ChassisSpeeds speeds) {
        if (averageTagDistance >= MAX_DISTANCE) return false;
        double driveVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double turnVelocity = Math.abs(speeds.omegaRadiansPerSecond);
        return driveVelocity <= MAX_DRIVE_SPEED && turnVelocity <= MAX_TURN_SPEED;
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
