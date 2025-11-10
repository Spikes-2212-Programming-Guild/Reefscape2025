package frc.robot.util.localization.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Represents a single vision measurement.
 *
 * @param standardDeviations the standardDeviations of the measurement
 * @param pose2d             the robot pose at the given timestamp
 * @param timestamp          the measurement timestamp
 * @author Itay Zadok
 */
public record VisionMeasurement(StandardDeviations standardDeviations,
                                Pose2d pose2d,
                                double timestamp
) {

}
