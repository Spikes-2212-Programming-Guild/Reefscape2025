package frc.robot.util;

import com.spikes2212.util.Limelight;
import edu.wpi.first.math.geometry.*;

/**
 * @link <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api">...</a>
 */
public class VisionService {

    private static final String LIMELIGHT_NAME = "limelight";

    private final Limelight limelight;

    private static VisionService instance;

    public static VisionService getInstance() {
        if (instance == null) {
            instance = new VisionService(LIMELIGHT_NAME);
        }
        return instance;
    }

    private VisionService(String limelightName) {
        limelight = new Limelight(limelightName);
    }

    /**
     * @return tag pose relative to camera coordinates.
     */
    public Pose2d getTargetRelativePose() {
        double[] result = limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[0]);
        if (limelight.getID() >= 0) {
            Translation2d translation2d = new Translation2d(result[0], result[1]);
            Rotation2d rotation2d = new Rotation2d(result[2]);
            return new Pose2d(translation2d, rotation2d);
        }
        return null;
    }

    /**
     * @return The total latency (capture latency + target latency).
     */
    public double getLatencySeconds() {
        try {
            double[] pose = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
            if (pose.length > 6) {
                // total latency is at index 6
                return (pose[6] / 1000); // convert ms to seconds
            } else {
                // array is too short to contain tag count
                return 0;
            }
        } catch (Exception e) {
            return 0;
        }
    }

    /**
     * @return The amount of visible april tags.
     */
    public int getTagCount() {
        try {
            double[] pose = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
            if (pose.length > 7) {
                // tag count is at index 7
                return (int) pose[7];
            } else {
                // array is too short to contain tag count
                return 0;
            }
        } catch (Exception e) {
            return 0;
        }
    }

    /**
     * @return the camera pose relative to field.
     */
    public Pose2d getCameraFieldPose() {
        double[] result = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[]{});
        if (limelight.getID() >= 0) {
            Translation3d translation3d = new Translation3d(result[0], result[1], result[2]);
            Rotation3d rotation3d = new Rotation3d(Math.toRadians(result[3]),
                    Math.toRadians(result[4]), Math.toRadians(result[5]));
            return new Pose3d(translation3d, rotation3d).toPose2d();
        }
        return null;
    }

    public boolean hasTarget() {
        return limelight.hasTarget();
    }
}
