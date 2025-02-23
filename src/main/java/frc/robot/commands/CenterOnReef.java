package frc.robot.commands;

import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class CenterOnReef extends Command {

    private static final double DISTANCE_FROM_TARGET = 0;

    private static final RootNamespace namespace = new RootNamespace("center on reef");
    private static final PIDSettings pidSettings = namespace.addPIDNamespace("center on reef");
    private static final FeedForwardSettings feedForwardSettings =
            namespace.addFeedForwardNamespace("center on reef", FeedForwardController.ControlMode.LINEAR_POSITION);

    private final Drivetrain drivetrain;
    private final boolean goLeft;

    private final PIDController pidController;
    private final FeedForwardController feedForwardController;

    private double lastTimeNotOnTarget;

    public CenterOnReef(Drivetrain drivetrain, boolean goLeft) {
        this.drivetrain = drivetrain;
        this.goLeft = goLeft;
        pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setTolerance(pidSettings.getTolerance());
        feedForwardController = new FeedForwardController(feedForwardSettings);
    }

    @Override
    public void execute() {
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setTolerance(pidSettings.getTolerance());
        feedForwardController.setGains(feedForwardSettings);
        // @TODO make sure this is correct
        double setpoint = goLeft ? DISTANCE_FROM_TARGET : -DISTANCE_FROM_TARGET;
        drivetrain.drive(0, pidController.calculate(drivetrain.getPose2d().getX(), setpoint) +
                feedForwardController.calculate(drivetrain.getPose2d().getX(), setpoint), 0, false, false, 0.02);
    }

    @Override
    public boolean isFinished() {
        if (!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return pidSettings.getWaitTime() >= lastTimeNotOnTarget - Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
