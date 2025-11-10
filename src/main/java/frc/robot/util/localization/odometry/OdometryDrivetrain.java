package frc.robot.util.localization.odometry;

/**
 * Represents a source of odometry data for pose estimation systems.
 * <p>
 * Any drivetrain that can provide a {@link OdometryMeasurement}
 *
 * @author Itay Zadok
 */
public interface OdometryDrivetrain {

    /**
     * Returns a snapshot of the current odometry state for the pose estimator
     *
     * @return a new {@link OdometryMeasurement} containing the current timestamp,
     * heading, and swerve module positions
     */
    OdometryMeasurement takeOdometryMeasurement();
}
