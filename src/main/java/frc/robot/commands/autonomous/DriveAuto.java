package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveAuto extends Command {

    private static final double DRIVE_SPEED = 0.5;
    private static final double DRIVE_TIME = 3;

    private final Drivetrain drivetrain;

    public DriveAuto(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        drivetrain.drive(0.0, DRIVE_SPEED, 0.0, false, false);
    }

    @Override
    public boolean isFinished() {
        return DRIVE_TIME >= Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
