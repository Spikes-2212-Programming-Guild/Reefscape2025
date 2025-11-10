package frc.robot.util.localization.odometry;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * A singleton used to schedule high or low frequency periodic tasks.
 *
 * @author Itay Zadok
 */
public class PeriodicTaskScheduler {

    private static volatile PeriodicTaskScheduler instance;
    private final TimedRobot timedRobot;

    private PeriodicTaskScheduler(TimedRobot timedRobot) {
        this.timedRobot = timedRobot;
    }

    /**
     * Initializes the global scheduler instance.
     * Must be called once during robot initialization.
     *
     * @param timedRobot the {@link TimedRobot} instance to attach tasks to
     * @throws IllegalStateException if the scheduler has already been initialized
     */
    public static void init(TimedRobot timedRobot) {
        if (instance != null) {
            throw new IllegalStateException("PeriodicTaskScheduler already initialized!");
        }
        instance = new PeriodicTaskScheduler(timedRobot);
    }

    /**
     * Returns the singleton instance of this scheduler.
     *
     * @return the {@link PeriodicTaskScheduler} instance
     * @throws IllegalStateException if {@link #init(TimedRobot)} has not yet been called
     */
    public static PeriodicTaskScheduler getInstance() {
        if (instance == null) {
            throw new IllegalStateException("PeriodicTaskScheduler not initialized! Call init first.");
        }
        return instance;
    }

    /**
     * Schedules a periodic task to run at a specified frequency.
     * Wraps {@link TimedRobot#addPeriodic(Runnable, double, double)} for convenience.
     *
     * @param task         the task to execute periodically
     * @param frequencyHz  how often to run the task, in Hertz (e.g., 100 → every 10ms)
     * @param delaySeconds the delay before the first execution, in seconds
     */
    public void schedule(Runnable task, double frequencyHz, double delaySeconds) {
        if (frequencyHz <= 0) {
            throw new IllegalArgumentException("Frequency must be positive.");
        }
        timedRobot.addPeriodic(task, 1.0 / frequencyHz, delaySeconds);
    }
}

