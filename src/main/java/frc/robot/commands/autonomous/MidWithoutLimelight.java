package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive;
import frc.robot.commands.ReleaseCoral;
import frc.robot.commands.RotateStorage;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.CoralJoint;

import java.util.function.Supplier;

public class MidWithoutLimelight extends SequentialCommandGroup {

    private static final Supplier<Double> DRIVE_SPEED = () -> 1.5;
    private static final double DRIVE_TIMEOUT = 4;

    public MidWithoutLimelight(Drivetrain drivetrain, CoralJoint coralJoint, Storage storage){
        addCommands(new ParallelCommandGroup(
                new Drive(drivetrain, DRIVE_SPEED, () -> 0.0, () -> 0.0, true, false,
                        false).withTimeout(DRIVE_TIMEOUT),
                new RotateStorage(coralJoint, CoralJoint.StoragePose.L2){
                    @Override
                    public void end(boolean interrupted){}
                }),
                new ReleaseCoral(storage),
                new RotateStorage(coralJoint, CoralJoint.StoragePose.RESTING));
    }
}
