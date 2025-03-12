package frc.robot.commands.autonomous;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveAtAngle;
import frc.robot.commands.ReleaseCoral;
import frc.robot.commands.RotateStorage;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.CoralJoint;

import java.util.function.Supplier;

public class DriveAndPlaceL1 extends SequentialCommandGroup {

    private static final Supplier<Double> DRIVE_SPEED = () -> 1.0;
    private static final double DRIVE_TIMEOUT = 3.0;

    public DriveAndPlaceL1(Drivetrain drivetrain, CoralJoint coralJoint, Storage storage) {
        addCommands(
                new MoveGenericSubsystem(coralJoint, -0.15).withTimeout(1),
                new DriveAtAngle(drivetrain, DRIVE_SPEED, () -> 0.0, () -> 0.0,
                        false, false).withTimeout(DRIVE_TIMEOUT),
                new DriveAtAngle(drivetrain, () -> -0.2, () -> 0.0, () -> 0.0, false, false).withTimeout(0.75),
                new RotateStorage(coralJoint, CoralJoint.StoragePose.L1),
                new ReleaseCoral(storage),
                new InstantCommand(() -> {
                    coralJoint.removeDefaultCommand();
                    coralJoint.finish();
                }));
    }
}
