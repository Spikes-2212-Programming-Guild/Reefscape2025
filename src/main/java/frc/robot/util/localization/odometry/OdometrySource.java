package frc.robot.util.localization.odometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * Represents a source of odometry data for pose estimation systems.
 * <p>
 * Any drivetrain or simulated system that can provide its current gyro heading,
 * swerve module positions, and timestamp can implement this interface to
 * supply odometry data to an {@link OdometryManager}.
 *
 * @author Itay Zadok
 */
public interface OdometrySource {

    /**
     * @return The timestamp (in seconds) of the current odometry reading.
     * Retrieved from {@code Timer.getFPGATimestamp()}.
     */
    double getTimestamp();

    /**
     * @return The current robot heading as a {@link Rotation2d}, measured by the gyro.
     */
    Rotation2d getHeading();

    /**
     * @return An array of {@link SwerveModulePosition} representing each module's wheel rotation and position.
     */
    SwerveModulePosition[] getModulePositions();

    /**
     * Creates a snapshot of the current odometry state.
     *
     * @return a new {@link OdometryMeasurement} containing the current timestamp,
     * heading, and swerve module positions
     */
    default OdometryMeasurement takeMeasurement() {
        return new OdometryMeasurement(getTimestamp(), getHeading(), getModulePositions());
    }

}

