package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorPositionManagement {

    private final int lastLevel;

    public ElevatorPositionManagement(int lastLevel) {
        this.lastLevel = lastLevel;
    }

    public int level(double speed, int lastLevel) {
        if (speed > 0) {
            return lastLevel + 1;
        }
        else if (speed < 0) {
            return lastLevel - 1;
        }
        return lastLevel;
    }

    public void changeLevel() {

    }
}
