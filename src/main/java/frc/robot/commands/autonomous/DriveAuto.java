package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class DriveAuto extends Drive {

    private static final Supplier<Double> DRIVE_SPEED = () -> 0.5;
    private static final double DRIVE_TIME = 3;

    public DriveAuto() {
        super(Drivetrain.getInstance(), DRIVE_SPEED, () -> 0.0, () -> 0.0, false, false, false);
    }

    @Override
    public boolean isFinished() {
        return DRIVE_TIME >= Timer.getFPGATimestamp();
    }
}
