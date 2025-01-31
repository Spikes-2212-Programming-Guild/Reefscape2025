package frc.robot;

public class RobotMap {

    public interface CAN {

        int FRONT_LEFT_DRIVE_TALON_FX = 1;
        int FRONT_RIGHT_DRIVE_TALON_FX = 4;
        int BACK_LEFT_DRIVE_TALON_FX = -1;
        int BACK_RIGHT_DRIVE_TALON_FX = -1;

        int FRONT_LEFT_TURN_SPARK_MAX = 2;
        int FRONT_RIGHT_TURN_SPARK_MAX = -1;
        int BACK_LEFT_TURN_SPARK_MAX = -1;
        int BACK_RIGHT_TURN_SPARK_MAX = -1;

        int FRONT_LEFT_ABSOLUTE_ENCODER = 3;
        int FRONT_RIGHT_ABSOLUTE_ENCODER = -1;
        int BACK_LEFT_ABSOLUTE_ENCODER = -1;
        int BACK_RIGHT_ABSOLUTE_ENCODER = -1;
    }
    
    public interface DIO {

    }
    
    public interface PWM {

    }
    
    public interface AIN {
    
    }

    public interface PCM {

    }
}
