package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class JustDrive extends SequentialCommandGroup {

    private static final Supplier<Double> DRIVE_SPEED = () -> 1.0;
    private static final double DRIVE_TIMEOUT = 3.0;

    public JustDrive(Drivetrain drivetrain) {
        addCommands(new Drive(drivetrain, DRIVE_SPEED, () -> 0.0, () -> 0.0, true, false, false).withTimeout(DRIVE_TIMEOUT));
    }
}
