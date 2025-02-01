package frc.robot.util;

import com.spikes2212.util.Limelight;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class VisionService extends Limelight {

    private final Limelight limelight;

    public VisionService(Limelight limelight) {
        this.limelight = limelight;
    }

    public Pose2d getRelativePose() {
        double[] result =  limelight.getValue("targetpose_robotspace").getDoubleArray();
        if (getID() >= 0) {
            Translation2d translation2d = new Translation2d(result[0], result[1]);
            Rotation2d rotation2d = new Rotation2d(result[2]);
            return new Pose2d(translation2d, rotation2d);
        }
        return null;
    }
}
