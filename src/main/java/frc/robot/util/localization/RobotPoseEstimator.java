package frc.robot.util.localization;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.util.localization.odometry.OdometryManager;
import frc.robot.util.localization.odometry.OdometryMeasurement;
import frc.robot.util.localization.odometry.PeriodicTaskScheduler;
import frc.robot.util.localization.vision.VisionManager;
import frc.robot.util.localization.vision.VisionMeasurement;

import java.util.function.Supplier;

/**
 * Centralized localization system combining odometry and vision data
 * to produce a continuous robot pose estimate via {@link SwerveDrivePoseEstimator}.
 *
 * <p>Should be updated periodically via {@link #periodic(VisionMeasurement)}.
 *
 * <p>Encapsulates both {@link OdometryManager} (high-frequency odometry)
 * and {@link VisionManager} (vision corrections).</p>
 *
 * @author Itay Zadok
 */
public class RobotPoseEstimator {

    private final SwerveDrivePoseEstimator poseEstimator;
    private final OdometryManager odometryManager;
    private final VisionManager visionManager;

    /**
     * Constructs a robot pose estimator combining odometry and vision.
     *
     * @param kinematics                  the drivetrain kinematics
     * @param gyroAngle                   the current gyro heading
     * @param modulePositions             the initial swerve module positions
     * @param initPose                    the initial field-relative pose
     * @param odometryMeasurementSupplier supplier that provides current odometry data
     * @param scheduler                   scheduler for periodic odometry sampling
     * @param odometryFrequencyHz         the frequency of the odometry measurements
     * @param odometryMeasurementLimit    limit on amount of stored odometry measurements to avoid overflow
     */
    public RobotPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle,
                              SwerveModulePosition[] modulePositions, Pose2d initPose,
                              Supplier<OdometryMeasurement> odometryMeasurementSupplier,
                              PeriodicTaskScheduler scheduler, int odometryFrequencyHz,
                              int odometryMeasurementLimit) {
        this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, initPose);
        this.visionManager = new VisionManager(poseEstimator);
        this.odometryManager = new OdometryManager(
                poseEstimator, odometryMeasurementSupplier, scheduler,
                odometryFrequencyHz, odometryMeasurementLimit
        );
    }

    /**
     * Updates the estimator with new odometry and vision measurements.
     * <p>Should be called once per robot loop.</p>
     *
     * @param visionMeasurement the latest vision update, or {@code null} if unavailable
     */
    public void periodic(VisionMeasurement visionMeasurement) {
        odometryManager.update();
        visionManager.update(visionMeasurement);
    }

    /**
     * @return the current estimated field-relative robot pose
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the estimator to a known pose, clearing all queued odometry data.
     *
     * @param newPose the new known field-relative pose
     */
    public void resetPose(Pose2d newPose) {
        odometryManager.resetPose(newPose);
    }

    /**
     * Samples the pose estimate at a specific timestamp, if available.
     *
     * @param timestamp the target time (seconds)
     * @return the interpolated pose, or {@code null} if unavailable
     */
    public Pose2d getEstimatedPoseAtTimestamp(double timestamp) {
        return poseEstimator.sampleAt(timestamp).orElse(null);
    }

    /**
     * @return the underlying {@link SwerveDrivePoseEstimator}
     */
    public SwerveDrivePoseEstimator getEstimator() {
        return poseEstimator;
    }
}
