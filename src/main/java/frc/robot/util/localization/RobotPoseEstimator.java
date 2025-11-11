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
 * High-level class that manages all robot localization systems.
 * Combines data from odometry and vision to produce a robot pose estimate using a {@link SwerveDrivePoseEstimator}.
 *
 * <p>It is supposed to be updated periodically: {@link #periodic}.</p>
 *
 * @author Itay Zadok
 */
public class RobotPoseEstimator {

    private final SwerveDrivePoseEstimator poseEstimator;
    private final OdometryManager odometry;
    private final VisionManager vision;

    public RobotPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle,
                              SwerveModulePosition[] modulePositions, Pose2d initPose,
                              Supplier<OdometryMeasurement> odometryMeasurementSupplier,
                              PeriodicTaskScheduler scheduler) {
        this.poseEstimator = new SwerveDrivePoseEstimator(
                kinematics, gyroAngle, modulePositions, initPose
        );
        this.vision = new VisionManager(poseEstimator);
        this.odometry = new OdometryManager(poseEstimator, odometryMeasurementSupplier, scheduler);
    }

    /**
     * Periodically updates the pose estimator with new odometry and vision data.
     * <p>
     * This method should be called regularly (e.g., once per robot loop)
     * </p>
     *
     * @param visionMeasurement the current vision measurement
     */
    public void periodic(VisionMeasurement visionMeasurement) {
        odometry.periodic();
        vision.periodic(visionMeasurement);
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
