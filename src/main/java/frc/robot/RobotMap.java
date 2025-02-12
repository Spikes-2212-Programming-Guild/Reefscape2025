package frc.robot;

public class RobotMap {

    public interface CAN {

      int ELEVATOR_MASTER_SPARK = -1;
      int ELEVATOR_SLAVE_SPARK = -1;
      int ALGAE_JOINT_SPARK = -1;
      int CORAL_JOINT_SPARK = -1;
      int GRIPPER_TALON = -1;
      int STORAGE_SPARK = -1;
    }
  
    public interface DIO {

        int ELEVATOR_TOP_LIMIT = -1;
        int ELEVATOR_BOTTOM_LIMIT = -1;
        int ALGAE_TOP_LIMIT = -1;
        int ALGAE_BOTTOM_LIMIT = -1;
        int CORAL_JOINT_TOP_LIMIT = -1;
        int CORAL_JOINT_BOTTOM_LIMIT = -1;
        int GRIPPER_LIMIT = -1;
        int STORAGE_INFRARED = -1;
    }

    public interface PWM {

    }

    public interface AIN {

    }

    public interface PCM {

    }
}
