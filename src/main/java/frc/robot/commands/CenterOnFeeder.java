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

    public enum PossiblePosses {

        FIRST(0, 0), SECOND(0, 0), THIRD(0, 0), FOURTH(0, 0), FIFTH(0, 0), SIXTH(0, 0), SEVENTH(0, 0);

        public final double xPose;
        public final double yPose;

        PossiblePosses(double xPose, double yPose) {
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
        double shortestDistance = getDistanceFrom(PossiblePosses.FIRST.xPose, PossiblePosses.FIRST.yPose);
        PossiblePosses pose = PossiblePosses.FIRST;
        for (PossiblePosses possiblePosses : PossiblePosses.values()) {
            if (shortestDistance > getDistanceFrom(possiblePosses.xPose, possiblePosses.yPose)) {
                shortestDistance = getDistanceFrom(possiblePosses.xPose, possiblePosses.yPose);
                pose = possiblePosses;
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
        if (!xPIDController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return pidSettings.getWaitTime() >= lastTimeNotOnTarget - Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
