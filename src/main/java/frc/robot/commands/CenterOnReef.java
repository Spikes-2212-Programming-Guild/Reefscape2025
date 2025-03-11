package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;

import java.util.concurrent.atomic.AtomicReference;

public class CenterOnReef extends SequentialCommandGroup {

    public CenterOnReef(Drivetrain drivetrain, OI.Side side) {
        AtomicReference<Double> rotationSetpoint = new AtomicReference<>(drivetrain.getYaw() - drivetrain.getRobotRelativePose().getRotation().getDegrees());
        addCommands(new RotateWithPID(drivetrain, rotationSetpoint::get).beforeStarting(new InstantCommand(() ->
                        rotationSetpoint.set(drivetrain.getYaw() - drivetrain.getRobotRelativePose().getRotation().getDegrees()))),
                new DriveToReef(drivetrain, side));
    }
}
