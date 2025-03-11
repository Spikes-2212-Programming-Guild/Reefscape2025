package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralJoint;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Storage;

public class PlaceOnReef extends SequentialCommandGroup {

    public enum Level {

        L1(Elevator.ElevatorLevel.L1, CoralJoint.StoragePose.L1), L2(Elevator.ElevatorLevel.L2, CoralJoint.StoragePose.L2),
        L3(Elevator.ElevatorLevel.L3, CoralJoint.StoragePose.L3);

        private final Elevator.ElevatorLevel elevatorLevel;
        private final CoralJoint.StoragePose storagePose;

        Level(Elevator.ElevatorLevel elevatorLevel, CoralJoint.StoragePose storagePose) {

            this.elevatorLevel = elevatorLevel;
            this.storagePose = storagePose;
        }
    }

    public PlaceOnReef(Elevator elevator, CoralJoint coralJoint, Storage storage, Level level) {
        addCommands(
                new ParallelCommandGroup(
                        new MoveToHeight(elevator, level.elevatorLevel),
                        new RotateStorage(coralJoint, level.storagePose)
                ),
                new ReleaseCoral(storage),
                new ParallelCommandGroup(
                        new MoveToHeight(elevator, Elevator.ElevatorLevel.BOTTOM),
                        new RotateStorage(coralJoint, CoralJoint.StoragePose.RESTING).withTimeout(0.5)
                ),
                new ParallelCommandGroup(
                        new InstantCommand(elevator::finish),
                        new InstantCommand(coralJoint::finish)
                )
        );
    }
}
