package frc.robot.util.localization.odometry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Supplier;


/**
 * Collects and applies high-frequency odometry measurements to a {@link SwerveDrivePoseEstimator}.
 *
 * <p>Odometry is sampled asynchronously using a {@link PeriodicTaskScheduler}
 * and applied sequentially during {@link #update()} calls.</p>
 *
 * @author Itay Zadok
 */
public class OdometryManager {

    private final int storedMeasurementsLimit;
    private final Queue<OdometryMeasurement> measurementsQueue = new ConcurrentLinkedQueue<>();
    private final SwerveDrivePoseEstimator estimator;
    private final Supplier<OdometryMeasurement> measurementSupplier;

    /**
     * Constructs a new OdometryManager.
     *
     * @param estimator               the shared pose estimator
     * @param measurementSupplier     supplier that provides odometry data
     * @param scheduler               scheduler for periodic sampling
     * @param frequencyHz             sampling rate
     * @param storedMeasurementsLimit limit on stored measurements to avoid overflow
     */
    public OdometryManager(SwerveDrivePoseEstimator estimator,
                           Supplier<OdometryMeasurement> measurementSupplier,
                           PeriodicTaskScheduler scheduler, int frequencyHz, int storedMeasurementsLimit) {
        this.estimator = estimator;
        this.measurementSupplier = measurementSupplier;
        this.storedMeasurementsLimit = storedMeasurementsLimit;
        scheduler.schedule(this::recordMeasurement, frequencyHz, 0);
    }

    /**
     * Applies all queued odometry samples to the estimator in order.
     * Should be called once per robot update loop.
     */
    public void update() {
        while (!measurementsQueue.isEmpty()) {
            OdometryMeasurement m = measurementsQueue.poll();
            estimator.updateWithTime(m.timestamp(), m.heading(), m.wheelPositions());
        }
    }

    /**
     * Records a new odometry measurement at high frequency.
     * This method is called automatically by the scheduler.
     */
    private void recordMeasurement() {
        OdometryMeasurement m = measurementSupplier.get();
        if (m == null) return;

        if (measurementsQueue.size() >= storedMeasurementsLimit)
            measurementsQueue.poll(); // drop oldest

        measurementsQueue.add(m);
    }

    /**
     * Resets the pose estimator and clears queued odometry data.
     *
     * @param newPose the new known robot pose
     */
    public void resetPose(Pose2d newPose) {
        OdometryMeasurement m = measurementSupplier.get();
        if (m == null) return;
        estimator.resetPosition(
                m.heading(),
                m.wheelPositions(),
                newPose
        );
        measurementsQueue.clear();
    }
}
