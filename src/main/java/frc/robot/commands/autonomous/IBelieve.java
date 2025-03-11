package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.CoralJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Storage;

public class IBelieve extends SequentialCommandGroup {

    public IBelieve(Drivetrain drivetrain, CoralJoint coralJoint, Storage storage) {
        addCommands(
                new DriveAndPlaceL1(drivetrain, coralJoint, storage),
                new Drive(drivetrain, () -> -1.0, () -> 0.0, () -> 0.0, true, false, false).withTimeout(3),
                new DriveToPose(drivetrain, new Pose2d(1.24, 0.97, Rotation2d.fromDegrees(102)))
        );
    }
}
