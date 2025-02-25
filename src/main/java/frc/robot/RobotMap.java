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

        int ELEVATOR_MASTER_SPARK = -1;
        int ELEVATOR_SLAVE_SPARK = -1;
        int ALGAE_JOINT_SPARK = 13;
        int CORAL_JOINT_TALON = 16;
        int GRIPPER_VICTOR = 17;
        int STORAGE_SPARK = 14;
    }

    public interface DIO {

        int ELEVATOR_TOP_LIMIT = -1;
        int ELEVATOR_BOTTOM_LIMIT = -1;
        int ALGAE_TOP_LIMIT = 0;
        int CORAL_JOINT_TOP_LIMIT = -1;
        int CORAL_JOINT_BOTTOM_LIMIT = 8;
        int GRIPPER_LIMIT = 5;
        int STORAGE_INFRARED = 3;
    }

    public interface PWM {

    }

    public interface AIN {

    }

    public interface PCM {

    }
}
