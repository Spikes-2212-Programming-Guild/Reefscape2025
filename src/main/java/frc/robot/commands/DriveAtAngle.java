package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class DriveAtAngle extends Command {

    private static final double DRIVE_ACCELERATION_LIMIT = 12;

    private final Drivetrain drivetrain;
    private final Supplier<Double> xSpeed;
    private final Supplier<Double> ySpeed;
    private final Supplier<Double> angle;
    private final boolean usePID;
    private final boolean limitAcceleration;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;

    private double time;

    public DriveAtAngle(Drivetrain drivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
                        Supplier<Double> angle, boolean usePID,
                        boolean limitAcceleration) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.angle = angle;
        this.usePID = usePID;
        this.limitAcceleration = limitAcceleration;
        xLimiter = new SlewRateLimiter(DRIVE_ACCELERATION_LIMIT);
        yLimiter = new SlewRateLimiter(DRIVE_ACCELERATION_LIMIT);
    }

    @Override
    public void initialize() {
        time = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double xSpeed = this.xSpeed.get();
        double ySpeed = this.ySpeed.get();

        if (limitAcceleration) {
            xSpeed = xLimiter.calculate(this.xSpeed.get());
            ySpeed = yLimiter.calculate(this.ySpeed.get());
        }

        drivetrain.stayAtAngle(xSpeed, ySpeed, angle.get(), usePID, Timer.getFPGATimestamp() - time);
        time = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
