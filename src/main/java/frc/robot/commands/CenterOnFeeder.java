package frc.robot.commands;

import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class CenterOnFeeder extends Command {

    public enum PossiblePose {

        FAR_LEFT(0, 0), MIDDLE_LEFT(0, 0), CLOSE_LEFT(0, 0),
        CENTER(0, 0), CLOSE_RIGHT(0, 0), MIDDLE_RIGHT(0, 0), FAR_RIGHT(0, 0);

        public final double xPose;
        public final double yPose;

        PossiblePose(double xPose, double yPose) {
            this.xPose = xPose;
            this.yPose = yPose;
        }
    }

    private static final RootNamespace namespace = new RootNamespace("center on feeder");
    private static final PIDSettings pidSettings = namespace.addPIDNamespace("center on feeder");
    private static final FeedForwardSettings feedForwardSettings =
            namespace.addFeedForwardNamespace("center on feeder",
                    FeedForwardController.ControlMode.LINEAR_POSITION);

    private final Drivetrain drivetrain;

    private final PIDController xPIDController;
    private final FeedForwardController xFeedForwardController;
    private final PIDController yPIDController;
    private final FeedForwardController yFeedForwardController;

    private double lastTimeNotOnTarget;

    public CenterOnFeeder(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
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
        double shortestDistance = getDistanceFrom(PossiblePose.FAR_LEFT.xPose, PossiblePose.FAR_LEFT.yPose);
        PossiblePose pose = PossiblePose.FAR_LEFT;
        for (PossiblePose possiblePose : PossiblePose.values()) {
            if (shortestDistance > getDistanceFrom(possiblePose.xPose, possiblePose.yPose)) {
                shortestDistance = getDistanceFrom(possiblePose.xPose, possiblePose.yPose);
                pose = possiblePose;
            }
        }
        drivetrain.drive(xPIDController.calculate(drivetrain.getPose2d().getX(), pose.xPose) +
                        xFeedForwardController.calculate(drivetrain.getPose2d().getX(), pose.xPose),
                yPIDController.calculate(drivetrain.getPose2d().getY(), pose.yPose) +
                        yFeedForwardController.calculate(drivetrain.getPose2d().getY(), pose.yPose),
                0, true, true, 0.02);
    }

    private double getDistanceFrom(double x, double y) {
        return Math.sqrt(Math.pow(x - drivetrain.getPose2d().getX(), 2) + Math.pow(y - drivetrain.getPose2d().getY(), 2));
    }

    @Override
    public boolean isFinished() {
        if (!xPIDController.atSetpoint() || !yPIDController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return pidSettings.getWaitTime() <= lastTimeNotOnTarget - Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
