package frc.robot.commands.district2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.MoveToHeight;
import frc.robot.commands.RotateAlgaeJointToTop;
import frc.robot.commands.RotateStorage;
import frc.robot.subsystems.AlgaeJoint;
import frc.robot.subsystems.CoralJoint;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.district2.District2CoralJoint;

public class District2Reset extends ParallelCommandGroup {

    public District2Reset(District2CoralJoint coralJoint, AlgaeJoint algaeJoint) {
        addCommands(new District2RotateStorage(coralJoint, District2CoralJoint.StoragePose.RESTING),
                new RotateAlgaeJointToTop(algaeJoint));
    }
}
