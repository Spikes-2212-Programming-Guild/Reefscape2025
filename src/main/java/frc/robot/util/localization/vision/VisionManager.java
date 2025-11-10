package frc.robot.util.localization.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class VisionManager {

    private final SwerveDrivePoseEstimator poseEstimator;

    public VisionManager(SwerveDrivePoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    public void periodic(VisionMeasurement measurement) {
        if (measurement == null) return;
        poseEstimator.addVisionMeasurement(
                measurement.pose2d(),
                measurement.timestamp(),
                measurement.standardDeviations().toMatrix()
        );
    }
}
