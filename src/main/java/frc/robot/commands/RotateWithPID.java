package frc.robot.commands;

import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class RotateWithPID extends Command {

    private static final RootNamespace namespace = new RootNamespace("rotate with pid");
    private static final PIDSettings pidSettings = namespace.addPIDNamespace("rotate with pid");

    private static final FeedForwardSettings feedForwardSettings = namespace.addFeedForwardNamespace("rotate with pid",
            FeedForwardSettings.EMPTY_FF_SETTINGS);

    private final Drivetrain drivetrain;
    private final Supplier<Double> setpoint;
    private final PIDController pidController;
    private final FeedForwardController feedForwardController;

    private double lastTimeNotOnTarget;

    public RotateWithPID(Drivetrain drivetrain, Supplier<Double> setpoint) {
        this.drivetrain = drivetrain;
        this.setpoint = setpoint;
        pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setTolerance(pidSettings.getTolerance());
        feedForwardController = new FeedForwardController(feedForwardSettings);
    }

    @Override
    public void execute() {
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setTolerance(pidSettings.getTolerance());
        feedForwardController.setGains(feedForwardSettings);
        drivetrain.drive(0, 0, pidController.calculate(drivetrain.getYaw(), setpoint.get()) +
                feedForwardController.calculate(drivetrain.getYaw(), setpoint.get()),
                false, false);
    }

    @Override
    public boolean isFinished() {
        if (!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return pidSettings.getWaitTime() >= Timer.getFPGATimestamp() - lastTimeNotOnTarget;
    }
}
