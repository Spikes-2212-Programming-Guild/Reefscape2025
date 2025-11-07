package frc.robot.util.localization.odometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * Represents a single odometry measure.
 *
 * @param timestamp      measurement timestamp
 * @param heading   gyro heading at the timestamp
 * @param wheelPositions swerve module wheel rotations & positions at the timestamp
 * @author Itay Zadok
 */
public record OdometryMeasurement(double timestamp,
                                  Rotation2d heading,
                                  SwerveModulePosition[] wheelPositions
) {

}
