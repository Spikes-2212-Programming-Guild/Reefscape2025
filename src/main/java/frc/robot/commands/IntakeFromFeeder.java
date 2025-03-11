package frc.robot.commands;

import com.spikes2212.dashboard.SpikesLogger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.CoralJoint;

public class IntakeFromFeeder extends SequentialCommandGroup {

    SpikesLogger logger = new SpikesLogger();

    public IntakeFromFeeder(Elevator elevator, CoralJoint coralJoint, Storage storage) {
        addCommands(new ParallelCommandGroup(
                        new MoveToHeight(elevator, Elevator.ElevatorLevel.FEEDER),
                        new RotateStorage(coralJoint, CoralJoint.StoragePose.INTAKE)
                ).andThen(logger.logCommand("finished")),
                new IntakeCoral(storage),
                new ParallelCommandGroup(
                        new MoveToHeight(elevator, Elevator.ElevatorLevel.BOTTOM),
                        new RotateStorage(coralJoint, CoralJoint.StoragePose.RESTING).withTimeout(0.5)
                ),
                new InstantCommand( () -> {
                    coralJoint.finish();
                    elevator.finish();
                })
        );
    }
}
