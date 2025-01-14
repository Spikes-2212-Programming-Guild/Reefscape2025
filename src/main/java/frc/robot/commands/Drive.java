package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class Drive extends Command {
    private final Drivetrain drivetrain;
    private final Supplier<Double> xSpeed;
    private final Supplier<Double> ySpeed;
    private final Supplier<Double> rotationsSpeed;
    private final boolean fieldRelative;
    private final boolean usePID;

    public Drive( Drivetrain drivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotationsSpeed, boolean fieldRelative, boolean usePID) {
        this.drivetrain = drivetrain;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotationsSpeed = rotationsSpeed;
        this.fieldRelative = fieldRelative;
        this.usePID = usePID;
    }

    @Override
    public void execute() {
        double xSpeed = this.xSpeed.get();
        double ySpeed = this.ySpeed.get();
        double rotationSpeed = this.rotationsSpeed.get();
        drivetrain.drive(xSpeed, ySpeed, rotationSpeed, fieldRelative, usePID);
    }

    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
