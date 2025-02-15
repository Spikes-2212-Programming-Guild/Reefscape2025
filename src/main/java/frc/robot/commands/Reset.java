package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeJoint;
import frc.robot.subsystems.CoralJoint;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Storage;

public class Reset extends SequentialCommandGroup {

    public Reset(Elevator elevator, CoralJoint coralJoint, AlgaeJoint algaeJoint) {
        addCommands(new ParallelCommandGroup(new MoveToHeight(elevator, Elevator.ElevatorLevel.BOTTOM),
                new RotateStorage(coralJoint, CoralJoint.StoragePose.RESTING),
                new RotateAlgaeJointToBottom(algaeJoint)));
    }
}
