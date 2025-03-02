package frc.robot.commands;

import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;

public class CenterOnReef extends Command {

    private static final double DISTANCE_FROM_TARGET = 0;

    private static final RootNamespace namespace = new RootNamespace("center on reef");
    private static final PIDSettings pidSettings = namespace.addPIDNamespace("center on reef");
    private static final FeedForwardSettings feedForwardSettings =
            namespace.addFeedForwardNamespace("center on reef", FeedForwardController.ControlMode.LINEAR_POSITION);

    private final OI.Side side;
    private final Drivetrain drivetrain;

    private final PIDController xPIDController;
    private final FeedForwardController xFeedForwardController;
    private final PIDController yPIDController;
    private final FeedForwardController yFeedForwardController;

    private double lastTimeNotOnTarget;

    public CenterOnReef(Drivetrain drivetrain, OI.Side side) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.side = side;
        xPIDController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        xPIDController.setTolerance(pidSettings.getTolerance());
        xFeedForwardController = new FeedForwardController(feedForwardSettings);
        yPIDController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        yPIDController.setTolerance(pidSettings.getTolerance());
        yFeedForwardController = new FeedForwardController(feedForwardSettings);
    }

    @Override
    public void execute() {
        xPIDController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        xPIDController.setTolerance(pidSettings.getTolerance());
        xFeedForwardController.setGains(feedForwardSettings);
        yPIDController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        yPIDController.setTolerance(pidSettings.getTolerance());
        yFeedForwardController.setGains(feedForwardSettings);
        // @TODO make sure this is correct
        double ySetpoint = side == OI.Side.LEFT ? DISTANCE_FROM_TARGET : -DISTANCE_FROM_TARGET;
        double xSetpoint = drivetrain.getPose2d().getX();
        drivetrain.drive(xPIDController.calculate(drivetrain.getPose2d().getX(), xSetpoint) +
                        xFeedForwardController.calculate(drivetrain.getPose2d().getX(), xSetpoint),
                yPIDController.calculate(drivetrain.getPose2d().getY(), ySetpoint) +
                        yFeedForwardController.calculate(drivetrain.getPose2d().getY(), ySetpoint),
                0, true, true, 0.02);
    }

    @Override
    public boolean isFinished() {
        if (!xPIDController.atSetpoint() && yPIDController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return pidSettings.getWaitTime() >= lastTimeNotOnTarget - Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
