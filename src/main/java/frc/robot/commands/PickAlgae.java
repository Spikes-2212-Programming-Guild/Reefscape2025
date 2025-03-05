package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeJoint;
import frc.robot.subsystems.Gripper;

public class PickAlgae extends SequentialCommandGroup {

    public PickAlgae(AlgaeJoint algaeJoint, Gripper gripper) {
        addCommands(new RotateAlgaeJointToBottom(algaeJoint), new IntakeAlgae(gripper));
    }
}
