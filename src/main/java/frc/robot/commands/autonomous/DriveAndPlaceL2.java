package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive;
import frc.robot.commands.ReleaseCoral;
import frc.robot.commands.district2.District2RotateStorage;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.district2.District2CoralJoint;

import java.util.function.Supplier;

public class DriveAndPlaceL2 extends SequentialCommandGroup {

    private static final Supplier<Double> DRIVE_SPEED = () -> 1.0;
    private static final double DRIVE_TIMEOUT = 4.0;

    public DriveAndPlaceL2(Drivetrain drivetrain, District2CoralJoint coralJoint, Storage storage) {
        addCommands(
                new Drive(drivetrain, DRIVE_SPEED, () -> 0.0, () -> 0.0, false, false, false).withTimeout(DRIVE_TIMEOUT),
                new District2RotateStorage(coralJoint, District2CoralJoint.StoragePose.L2) {
                    @Override
                    public void end(boolean interrupted) {
                    }
                },
                new ReleaseCoral(storage),
                new InstantCommand(coralJoint::finish));
    }
}
