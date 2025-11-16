package frc.robot.util.localization.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Represents the uncertainty (standard deviation) of a robot pose estimate.
 * <p>
 * Larger standard deviation values indicate greater uncertainty (less trust)
 * in the measured position or heading. These values are typically used
 * to weight sensor data (e.g., odometry or vision) when fusing measurements.
 *
 * @param translationStdDev the standard deviation (in meters) for the robot’s position
 * @param rotationStdDev    the standard deviation (in radians) for the robot’s heading
 * @author Itay Zadok
 */
public record StandardDeviations(double translationStdDev, double rotationStdDev) {

    public Matrix<N3, N1> toMatrix() {
        return VecBuilder.fill(
                translationStdDev,
                translationStdDev,
                rotationStdDev
        );
    }
}
