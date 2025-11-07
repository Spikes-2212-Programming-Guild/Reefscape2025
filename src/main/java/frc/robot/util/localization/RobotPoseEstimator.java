package frc.robot.util.localization;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.localization.odometry.OdometryManager;
import frc.robot.util.localization.odometry.OdometrySource;
import frc.robot.util.localization.odometry.PeriodicTaskScheduler;
import frc.robot.util.localization.vision.VisionManager;

/**
 * High-level class that manages all robot localization systems.
 * Combines data from odometry and vision to produce an accurate, latency-compensated
 * robot pose estimate using a {@link SwerveDrivePoseEstimator}.
 *
 * <p>It is supposed to be updated periodically: {@link #periodic(double, ChassisSpeeds)}.</p>
 *
 * @author Itay Zadok
 */
public class RobotPoseEstimator {

    private final SwerveDrivePoseEstimator poseEstimator;
    private final OdometryManager odometry;
    private final VisionManager vision;

    /**
     * Constructs a {@code RobotPoseEstimator}.
     *
     * @param estimator the {@link SwerveDrivePoseEstimator} used for pose fusion
     * @param source    the odometry data source (e.g., drivetrain)
     * @param scheduler the scheduler used for high-frequency odometry sampling
     */
    public RobotPoseEstimator(SwerveDrivePoseEstimator estimator, OdometrySource source,
                              PeriodicTaskScheduler scheduler) {
        this.poseEstimator = estimator;
        this.vision = new VisionManager(estimator);
        this.odometry = new OdometryManager(estimator, source, scheduler);
    }

    /**
     * Periodically updates the pose estimator with new odometry and vision data.
     * <p>
     * This method should be called regularly (e.g., once per robot loop)
     * to maintain an up-to-date pose estimate.
     * </p>
     *
     * @param gyroYaw    current gyro yaw angle
     * @param robotSpeed current robot-relative chassis speeds
     */
    public void periodic(double gyroYaw, ChassisSpeeds robotSpeed) {
        odometry.periodic();
        vision.periodic(gyroYaw, robotSpeed);
    }

    /**
     * Returns the most recent estimated robot pose.
     *
     * @return the fused {@link Pose2d} from odometry and vision
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the estimator’s current pose to a new known position.
     * <p>
     * This also clears all pending odometry measurements.
     * </p>
     *
     * @param newPose the new {@link Pose2d} to reset to
     */
    public void resetPose(Pose2d newPose) {
        odometry.resetPose(newPose);
    }
}
