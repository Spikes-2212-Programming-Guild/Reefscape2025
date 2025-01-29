package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class Drive extends Command {

    private final double DRIVE_ACCELERATION_LIMIT = 2;
    private final double TURN_ACCELERATION_LIMIT = 4;

    private final Drivetrain drivetrain;
    private final Supplier<Double> xSpeed;
    private final Supplier<Double> ySpeed;
    private final Supplier<Double> rotationsSpeed;
    private final boolean fieldRelative;
    private final boolean usePID;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter rotationLimiter;

    public Drive(Drivetrain drivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
                 Supplier<Double> rotationsSpeed, boolean fieldRelative, boolean usePID) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotationsSpeed = rotationsSpeed;
        this.fieldRelative = fieldRelative;
        this.usePID = usePID;
        xLimiter = new SlewRateLimiter(DRIVE_ACCELERATION_LIMIT);
        yLimiter = new SlewRateLimiter(DRIVE_ACCELERATION_LIMIT);
        rotationLimiter = new SlewRateLimiter(TURN_ACCELERATION_LIMIT);
    }

    @Override
    public void execute() {
        double xSpeed = xLimiter.calculate(this.xSpeed.get());
        double ySpeed = yLimiter.calculate(this.ySpeed.get());
        double rotationSpeed = rotationLimiter.calculate(this.rotationsSpeed.get());
        drivetrain.drive(xSpeed, ySpeed, rotationSpeed, fieldRelative, usePID);
    }

    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
