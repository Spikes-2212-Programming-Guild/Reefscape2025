package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeJoint;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class PickAlgaeFloar extends SequentialCommandGroup {

    public PickAlgaeFloar(Elevator elevator, AlgaeJoint algaeJoint, Gripper gripper) {
        addCommands(new ParallelCommandGroup(new MoveToHeight(elevator, Elevator.ElevatorLevel.BOTTOM),
                new RotateAlgaeJointToBottom(algaeJoint)), new IntakeAlgae(gripper));
    }
}
