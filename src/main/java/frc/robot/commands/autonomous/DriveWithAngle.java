package frc.robot.commands.autonomous;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.SpikesLogger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveAtAngle;
import frc.robot.commands.ReleaseCoral;
import frc.robot.commands.RotateStorage;
import frc.robot.subsystems.CoralJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Storage;

import java.util.function.Supplier;

public class DriveWithAngle extends SequentialCommandGroup {

    private static final Supplier<Double> DRIVE_SPEED_WITH_ANGLE_FORWARD = () -> 1.0;
    private static final Supplier<Double> DRIVE_SPEED_WITH_ANGLE_BACKWARD = () -> -0.2;
    private static final Supplier<Double> DRIVE_SPEED_FORWARD = () -> 0.7;
    private static final double DRIVE_TIMEOUT_WITH_ANGLE_FORWARD = 3;
    private static final double DRIVE_TIMEOUT_WITH_ANGLE_BACKWARD = 0.75;
    private static final double DRIVE_TIMEOUT_FORWARD = 2;

    private final SpikesLogger logger = new SpikesLogger();

    public DriveWithAngle(Drivetrain drivetrain, CoralJoint coralJoint, Storage storage) {
        addCommands(
                new MoveGenericSubsystem(coralJoint, -0.15).withTimeout(1),
                new DriveAtAngle(drivetrain, DRIVE_SPEED_WITH_ANGLE_FORWARD, () -> 0.0, () -> 0.0, false,
                        false).withTimeout(DRIVE_TIMEOUT_WITH_ANGLE_FORWARD),
                new Drive(drivetrain, DRIVE_SPEED_FORWARD, () -> 0.0, () -> 0.0, false,
                        false, false).withTimeout(DRIVE_TIMEOUT_FORWARD),
                new InstantCommand(drivetrain::resetGyro),
                new WaitCommand(1),
                new DriveAtAngle(drivetrain, DRIVE_SPEED_WITH_ANGLE_BACKWARD, () -> 0.0, () -> 0.0, false,
                        false).withTimeout(DRIVE_TIMEOUT_WITH_ANGLE_BACKWARD).andThen(logger.logCommand("drive backwards")),
                new RotateStorage(coralJoint, CoralJoint.StoragePose.L1),
                new ReleaseCoral(storage).andThen(logger.logCommand("released coral")),
                new InstantCommand(() -> {
                    coralJoint.removeDefaultCommand();
                    coralJoint.finish();
                }));
    }
}
