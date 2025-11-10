package frc.robot.util.localization.odometry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Manages odometry data collection and application for a {@link SwerveDrivePoseEstimator}.
 * <p>
 * This class records high-frequency odometry snapshots from an {@link OdometryDrivetrain}
 * and applies them sequentially to the pose estimator.
 * </p>
 *
 * <p><b>Usage:</b> Call {@link #periodic()} periodically.</p>
 *
 * @author Itay Zadok
 */
public class OdometryManager {

    public static final int ODOMETRY_FREQUENCY_HZ = 100;
    private static final int MAX_QUEUE_SIZE = 1000;

    private final Queue<OdometryMeasurement> measures = new ConcurrentLinkedQueue<>();
    private final SwerveDrivePoseEstimator estimator;
    private final OdometryDrivetrain drivetrain;

    /**
     * Constructs a {@code OdometryManager}.
     *
     * @param estimator  the {@link SwerveDrivePoseEstimator} used for pose estimation
     * @param drivetrain the drivetrain used to get odometry data
     * @param scheduler  the scheduler used for high-frequency odometry sampling
     */
    public OdometryManager(SwerveDrivePoseEstimator estimator, OdometryDrivetrain drivetrain,
                           PeriodicTaskScheduler scheduler) {
        this.estimator = estimator;
        this.drivetrain = drivetrain;
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
     * Captures a new odometry snapshot from the {@link OdometryDrivetrain} and stores it in the queue.
     * <p>
     * This method runs automatically at a high frequency (e.g., 100 Hz) to provide time-accurate motion data.
     * The queue is bounded by {@link #MAX_QUEUE_SIZE} to prevent memory overflow.
     * </p>
     */
    private void recordMeasurement() {
        if (measures.size() >= MAX_QUEUE_SIZE) measures.poll(); // prevent overflow
        measures.add(drivetrain.takeOdometryMeasurement());
    }

    /**
     * Resets the pose estimator and clears all queued odometry data.
     *
     * @param newPose the new known pose to reset the estimator to
     */
    public void resetPose(Pose2d newPose) {
        OdometryMeasurement measurement = drivetrain.takeOdometryMeasurement();
        estimator.resetPosition(
                measurement.heading(),
                measurement.wheelPositions(),
                newPose
        );
        measures.clear();
    }
}
