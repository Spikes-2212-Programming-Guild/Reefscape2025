package frc.robot.commands.district2;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveToHeight;
import frc.robot.commands.ReleaseCoral;
import frc.robot.commands.RotateStorage;
import frc.robot.subsystems.CoralJoint;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.district2.District2CoralJoint;

public class District2PlaceOnReef extends SequentialCommandGroup {

    public District2PlaceOnReef(District2CoralJoint coralJoint, Storage storage, District2CoralJoint.StoragePose level) {
        addCommands(
                new District2RotateStorage(coralJoint, level),
                new ReleaseCoral(storage),
                new District2RotateStorage(coralJoint, District2CoralJoint.StoragePose.INTAKE)
        );
    }
}
