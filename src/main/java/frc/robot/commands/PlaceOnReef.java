package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralJoint;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Storage;

public class PlaceOnReef extends SequentialCommandGroup {

    public PlaceOnReef(Elevator elevator, CoralJoint coralJoint, Storage storage, Elevator.ElevatorLevel level) {
        addCommands(new ParallelCommandGroup(
                        new MoveToHeight(elevator, level),
                        new RotateStorage(coralJoint, CoralJoint.StoragePose.PLACEMENT)),
                new ReleaseCoral(storage), new ParallelCommandGroup(
                        new MoveToHeight(elevator, Elevator.ElevatorLevel.FEEDER),
                        new RotateStorage(coralJoint, CoralJoint.StoragePose.INTAKE)));
    }
}
