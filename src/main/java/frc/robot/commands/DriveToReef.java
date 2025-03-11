package frc.robot.commands;

import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class DriveToReef extends Command {

    private static final RootNamespace namespace = new RootNamespace("center on reef");
    private static final PIDSettings pidSettings = namespace.addPIDNamespace("center on reef",
            new PIDSettings(3, 0, 0, 0, 0.02, 0.3));
    private static final FeedForwardSettings feedForwardSettings =
            namespace.addFeedForwardNamespace("center on reef",
                    new FeedForwardSettings(0.06, 0, 0, FeedForwardController.ControlMode.LINEAR_POSITION));

    private final Supplier<Double> DISTANCE_X = namespace.addConstantDouble("x setpoint", -0.4);
    private final Supplier<Double> DISTANCE_Y_LEFT = namespace.addConstantDouble("left y setpoint", -0.27);
    private final Supplier<Double> DISTANCE_Y_RIGHT = namespace.addConstantDouble("right y setpoint", 0.4);

    private final OI.Side side;
    private final Drivetrain drivetrain;

    private final PIDController xPIDController;
    private final FeedForwardController xFeedForwardController;
    private final PIDController yPIDController;
    private final FeedForwardController yFeedForwardController;

    private double ySetpoint;
    private double initialRotation;
    private double lastTimeNotOnTarget;

    public DriveToReef(Drivetrain drivetrain, OI.Side side) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.side = side;
        xPIDController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        xPIDController.setTolerance(pidSettings.getTolerance());
        xFeedForwardController = new FeedForwardController(feedForwardSettings);
        yPIDController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        yPIDController.setTolerance(pidSettings.getTolerance());
        yFeedForwardController = new FeedForwardController(feedForwardSettings);
        ySetpoint = side == OI.Side.LEFT ? DISTANCE_Y_LEFT.get() : DISTANCE_Y_RIGHT.get();
        initialRotation = drivetrain.getYaw();
        namespace.putBoolean("x on target", () -> Math.abs(DISTANCE_X.get() - drivetrain.getRobotRelativePose().getX()) >= pidSettings.getTolerance());
        namespace.putBoolean("y on target", () -> Math.abs(DISTANCE_Y_LEFT.get() - drivetrain.getRobotRelativePose().getY()) >= pidSettings.getTolerance());
        namespace.putNumber("robot x", () -> drivetrain.getRobotRelativePose().getX());
        namespace.putNumber("robot y", () -> drivetrain.getRobotRelativePose().getY());
        namespace.putNumber("x tolerance", xPIDController::getErrorTolerance);
        namespace.putNumber("y tolerance", yPIDController::getErrorTolerance);
    }

    @Override
    public void initialize() {
        initialRotation = drivetrain.getYaw();
    }

    @Override
    public void execute() {
        xPIDController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        xPIDController.setTolerance(pidSettings.getTolerance());
        xFeedForwardController.setGains(feedForwardSettings);
        yPIDController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        yPIDController.setTolerance(pidSettings.getTolerance());
        yFeedForwardController.setGains(feedForwardSettings);
        ySetpoint = side == OI.Side.LEFT ? DISTANCE_Y_LEFT.get() : DISTANCE_Y_RIGHT.get();
//        drivetrain.drive(xPIDController.calculate(drivetrain.getPose2d().getX(), DISTANCE_X) +
//                        xFeedForwardController.calculate(drivetrain.getPose2d().getX(), DISTANCE_X),
        drivetrain.drive(0,
                yPIDController.calculate(drivetrain.getRobotRelativePose().getY(), ySetpoint) +
                        yFeedForwardController.calculate(drivetrain.getRobotRelativePose().getY(), ySetpoint),
                0, true, false, 0.02, false, Rotation2d.fromDegrees(drivetrain.getYaw() - initialRotation));
    }

    @Override
    public boolean isFinished() {
        if (
//                Math.abs(DISTANCE_X - drivetrain.getPose2d().getX()) >= pidSettings.getTolerance() ||
                        Math.abs(ySetpoint - drivetrain.getRobotRelativePose().getY()) >= pidSettings.getTolerance()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return pidSettings.getWaitTime() <= Timer.getFPGATimestamp() - lastTimeNotOnTarget;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
