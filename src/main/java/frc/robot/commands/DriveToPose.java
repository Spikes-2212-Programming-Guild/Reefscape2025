package frc.robot.commands;

import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveToPose extends Command {

    private static final RootNamespace namespace = new RootNamespace("drive to pose");
    private static final PIDSettings translationPIDSettings = namespace.addPIDNamespace("translation",
            new PIDSettings(3, 0, 0, 0, 0.02, 0.3));
    private static final FeedForwardSettings translationFeedForwardSettings =
            namespace.addFeedForwardNamespace("translation",
                    new FeedForwardSettings(0.06, 0, 0, FeedForwardController.ControlMode.LINEAR_POSITION));
    private static final PIDSettings rotationPIDSettings = namespace.addPIDNamespace("rotation",
            new PIDSettings(0.06, 0, 0.001, 0, 0.5, 0.5));
    private static final FeedForwardSettings rotationFeedForwardSettings =
            namespace.addFeedForwardNamespace("rotation",
                    new FeedForwardSettings(0.13, 0, 0, 0, FeedForwardController.ControlMode.LINEAR_POSITION));

    private final Drivetrain drivetrain;
    private final PIDController xPIDController;
    private final PIDController yPIDController;
    private final PIDController rotationPIDController;
    private final FeedForwardController xFeedForwardController;
    private final FeedForwardController yFeedForwardController;
    private final FeedForwardController rotationFeedForwardController;

    private final Pose2d setpoint;

    private double xLastTimeNotOnTarget;
    private double yLastTimeNotOnTarget;
    private double rotationLastTimeNotOnTarget;

    public DriveToPose(Drivetrain drivetrain, Pose2d setpoint) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        xPIDController = new PIDController(translationPIDSettings.getkP(), translationPIDSettings.getkI(), translationPIDSettings.getkD());
        xPIDController.setTolerance(translationPIDSettings.getTolerance());
        yPIDController = new PIDController(translationPIDSettings.getkP(), translationPIDSettings.getkI(), translationPIDSettings.getkD());
        yPIDController.setTolerance(translationPIDSettings.getTolerance());
        rotationPIDController = new PIDController(rotationPIDSettings.getkP(), rotationPIDSettings.getkI(),
                rotationPIDSettings.getkD());
        rotationPIDController.setTolerance(rotationPIDSettings.getTolerance());
        xFeedForwardController = new FeedForwardController(translationFeedForwardSettings);
        yFeedForwardController = new FeedForwardController(translationFeedForwardSettings);
        rotationFeedForwardController = new FeedForwardController(rotationFeedForwardSettings);
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        xLastTimeNotOnTarget = Timer.getFPGATimestamp();
        yLastTimeNotOnTarget = Timer.getFPGATimestamp();
        rotationLastTimeNotOnTarget = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        xPIDController.setPID(translationPIDSettings.getkP(), translationPIDSettings.getkI(), translationPIDSettings.getkD());
        xPIDController.setTolerance(translationPIDSettings.getTolerance());
        yPIDController.setPID(translationPIDSettings.getkP(), translationPIDSettings.getkI(), translationPIDSettings.getkD());
        yPIDController.setTolerance(translationPIDSettings.getTolerance());
        rotationPIDController.setPID(rotationPIDSettings.getkP(), rotationPIDSettings.getkI(), rotationPIDSettings.getkD());
        rotationPIDController.setTolerance(rotationPIDSettings.getTolerance());
        xFeedForwardController.setGains(translationFeedForwardSettings);
        yFeedForwardController.setGains(translationFeedForwardSettings);
        rotationFeedForwardController.setGains(rotationFeedForwardSettings);
//        drivetrain.drive(xPIDController.calculate(drivetrain.getFieldRelativePose().getX(), setpoint.getX()),
//                yPIDController.calculate(drivetrain.getFieldRelativePose().getY(), setpoint.getY()),
//                rotationPIDController.calculate(drivetrain.getFieldRelativePose().getRotation().getDegrees(),
//                        setpoint.getRotation().getDegrees()),
//                true, false, 0.02);
    }

    @Override
    public boolean isFinished() {
        if (!xPIDController.atSetpoint() || !yPIDController.atSetpoint() || !rotationPIDController.atSetpoint()) {
            xLastTimeNotOnTarget = Timer.getFPGATimestamp();
            yLastTimeNotOnTarget = Timer.getFPGATimestamp();
            rotationLastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        boolean xFinished = translationPIDSettings.getWaitTime() >= Timer.getFPGATimestamp() - xLastTimeNotOnTarget;
        boolean yFinished = translationPIDSettings.getWaitTime() >= Timer.getFPGATimestamp() - yLastTimeNotOnTarget;
        boolean rotationFinished = rotationPIDSettings.getWaitTime() >= Timer.getFPGATimestamp() - rotationLastTimeNotOnTarget;
        return xFinished && yFinished && rotationFinished;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
