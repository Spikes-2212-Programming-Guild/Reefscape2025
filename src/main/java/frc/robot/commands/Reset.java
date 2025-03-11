package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CoralJoint;
import frc.robot.subsystems.Elevator;

public class Reset extends ParallelCommandGroup {

    public Reset(Elevator elevator, CoralJoint coralJoint) {
        addCommands(new MoveToHeight(elevator, Elevator.ElevatorLevel.L1).andThen(
                new InstantCommand(coralJoint::removeDefaultCommand).andThen(new InstantCommand(coralJoint::finish))),
                new RotateStorage(coralJoint, CoralJoint.StoragePose.RESTING));
    }
}
