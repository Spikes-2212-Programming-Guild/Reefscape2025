package frc.robot.util.localization.odometry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Manages odometry data collection and application for a {@link SwerveDrivePoseEstimator}.
 * <p>
 * This class records high-frequency odometry snapshots from an {@link OdometrySource}
 * and applies them sequentially to the pose estimator to maintain accurate localization.
 * It smooths out potential timestamp mismatches between drivetrain updates and pose estimation.
 * </p>
 *
 * <p><b>Usage:</b> Call {@link #periodic()} periodically (e.g. in the drivetrain's periodic loop).</p>
 *
 * @author Itay Zadok
 */
public class OdometryManager {

    private static final double ODOMETRY_FREQUENCY_HZ = 250;
    private static final int MAX_QUEUE_SIZE = 1000;

    private final Queue<OdometryMeasurement> measures = new ConcurrentLinkedQueue<>();
    private final SwerveDrivePoseEstimator estimator;
    private final OdometrySource source;

    /**
     * Constructs a {@code OdometryManager}.
     *
     * @param estimator the {@link SwerveDrivePoseEstimator} used for pose estimation
     * @param source    the odometry data source (e.g., drivetrain)
     * @param scheduler the scheduler used for high-frequency odometry sampling
     */
    public OdometryManager(SwerveDrivePoseEstimator estimator,
                           OdometrySource source, PeriodicTaskScheduler scheduler) {
        this.estimator = estimator;
        this.source = source;
        scheduler.schedule(this::recordMeasurement, ODOMETRY_FREQUENCY_HZ, 0);
    }

    /**
     * Applies all queued odometry measurements to the pose estimator in chronological order.
     * <p>
     * This should be called periodically from the localization update loop.
     * </p>
     */
    public void periodic() {
        while (!measures.isEmpty()) {
            OdometryMeasurement m = measures.poll();
            estimator.updateWithTime(m.timestamp(), m.heading(), m.wheelPositions());
        }
    }

    /**
     * Captures a new odometry snapshot from the {@link OdometrySource} and stores it in the queue.
     * <p>
     * This method runs automatically at a high frequency (e.g., 250 Hz) to provide time-accurate motion data.
     * The queue is bounded by {@link #MAX_QUEUE_SIZE} to prevent memory growth.
     * </p>
     */
    private void recordMeasurement() {
        if (measures.size() >= MAX_QUEUE_SIZE) measures.poll(); // prevent overflow
        measures.add(source.takeMeasurement());
    }

    /**
     * Resets the pose estimator and clears all queued odometry data.
     *
     * @param newPose the new known pose to reset the estimator to
     */
    public void resetPose(Pose2d newPose) {
        estimator.resetPosition(
                source.getHeading(),
                source.getModulePositions(),
                newPose
        );
        measures.clear();
    }
}
