package frc.robot;

public class RobotMap {

    public interface CAN {

        int FRONT_LEFT_DRIVE_TALON_FX = 1;
        int FRONT_RIGHT_DRIVE_TALON_FX = 4;
        int BACK_LEFT_DRIVE_TALON_FX = 7;
        int BACK_RIGHT_DRIVE_TALON_FX = 10;

        int FRONT_LEFT_TURN_SPARK_MAX = 2;
        int FRONT_RIGHT_TURN_SPARK_MAX = 5;
        int BACK_LEFT_TURN_SPARK_MAX = 8;
        int BACK_RIGHT_TURN_SPARK_MAX = 11;

        int FRONT_LEFT_ABSOLUTE_ENCODER = 3;
        int FRONT_RIGHT_ABSOLUTE_ENCODER = 6;
        int BACK_LEFT_ABSOLUTE_ENCODER = 9;
        int BACK_RIGHT_ABSOLUTE_ENCODER = 12;

        int DRIVETRAIN_PIGEON = 15;

        int ELEVATOR_MASTER_SPARK = 18;
        int ELEVATOR_SLAVE_SPARK = 19;
        int ALGAE_JOINT_SPARK = 13;
        int CORAL_JOINT_TALON = 16;
        int STORAGE_SPARK = 14;
    }

    public interface DIO {

        int ELEVATOR_TOP_LIMIT = 0;
        int ELEVATOR_BOTTOM_LIMIT = 5;
        int CORAL_JOINT_BOTTOM_LIMIT = 3;
        int STORAGE_INFRARED = 2;
    }

    public interface PWM {

    }

    public interface AIN {

    }

    public interface PCM {

    }
}
