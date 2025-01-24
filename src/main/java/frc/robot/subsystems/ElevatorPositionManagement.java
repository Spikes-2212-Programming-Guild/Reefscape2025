package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorPositionManagement {

    private final SparkMax elevator;

    public ElevatorPositionManagement(SparkMax elevator) {
        this.elevator = elevator;
    }

    public int getLevel(double speed, int lastLevel) {
        if (speed > 0) {
            return lastLevel + 1;
        }
        else if (speed < 0) {
            return lastLevel - 1;
        }
        return lastLevel;
    }

    public void changeLevel(double speed, int wantedHeight) {
    }
}
