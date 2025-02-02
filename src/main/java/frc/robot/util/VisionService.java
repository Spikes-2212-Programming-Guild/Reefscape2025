package frc.robot.util;

import com.spikes2212.util.Limelight;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class VisionService {

    private final Limelight limelight;

    private static VisionService instance;

    public static VisionService getInstance() {
        if (instance == null) {
            instance = new VisionService(new Limelight("limelight"));
        }
        return instance;
    }

    public VisionService(Limelight limelight) {
        this.limelight = limelight;
    }

    public Pose2d getRelativePose() {
        double[] result =  limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[0]);
        if (limelight.getID() >= 0) {
            Translation2d translation2d = new Translation2d(result[0], result[1]);
            Rotation2d rotation2d = new Rotation2d(result[2]);
            return new Pose2d(translation2d, rotation2d);
        }
        return null;
    }

    public Pose2d getFieldRelativePose() {
        return limelight.getRobotPose().toPose2d();
    }
}
