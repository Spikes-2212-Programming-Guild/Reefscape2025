package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.Drive;
import frc.robot.subsystems.swerve.Drivetrain;

import java.util.function.Supplier;

public class JustDrive extends Drive {

    private static final Supplier<Double> DRIVE_SPEED = () -> 1.0;
    private static final double DRIVE_TIME = 3;

    public JustDrive() {
        super(Drivetrain.getInstance(), DRIVE_SPEED, () -> 0.0, () -> 0.0, false, false, false);
    }

    @Override
    public boolean isFinished() {
        return DRIVE_TIME >= Timer.getFPGATimestamp();
    }
}
