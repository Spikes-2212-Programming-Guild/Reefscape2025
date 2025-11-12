package frc.robot.util.localization.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

/**
 * Handles applying vision-based pose corrections to a {@link SwerveDrivePoseEstimator}.
 *
 * @author Itay Zadok
 */
public record VisionManager(SwerveDrivePoseEstimator estimator) {

    /**
     * Applies a vision measurement if valid.
     *
     * @param measurement the vision measurement, or {@code null} if unavailable
     */
    public void update(VisionMeasurement measurement) {
        if (measurement == null) return;
        estimator.addVisionMeasurement(
                measurement.pose2d(),
                measurement.timestamp(),
                measurement.standardDeviations().toMatrix()
        );
    }
}
