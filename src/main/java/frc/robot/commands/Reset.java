package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.AlgaeJoint;
import frc.robot.subsystems.CoralJoint;
import frc.robot.subsystems.Elevator;

public class Reset extends ParallelCommandGroup {

    public Reset(Elevator elevator, CoralJoint coralJoint, AlgaeJoint algaeJoint) {
        addCommands(new MoveToHeight(elevator, Elevator.ElevatorLevel.BOTTOM),
                new RotateStorage(coralJoint, CoralJoint.StoragePose.RESTING),
                new RotateAlgaeJointToTop(algaeJoint));
    }
}
