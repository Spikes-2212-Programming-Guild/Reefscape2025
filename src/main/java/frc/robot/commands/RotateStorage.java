package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystemWithPID;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.CoralJoint;

import java.util.function.Supplier;

public class RotateStorage extends MoveGenericSubsystemWithPID {

    private static final RootNamespace namespace = new RootNamespace("district 2 rotate storage");
    private static final PIDSettings intakePIDSettings = namespace.addPIDNamespace("intake rotate storage",
            new PIDSettings(0.2, 0.0, 0.0, 30, 3, 0.5));
    private static final FeedForwardSettings intakeFeedForwardSettings = namespace.addFeedForwardNamespace("intake rotate storage",
            new FeedForwardSettings(0.006, 0, 0, 0.026, FeedForwardController.ControlMode.ANGULAR_POSITION));
    private static final PIDSettings outtakePIDSettings = namespace.addPIDNamespace("outtake rotate storage",
            new PIDSettings(0.23, 0.0, 0.0, 4, 4, 0.5));
    private static final FeedForwardSettings outtakeFeedForwardSettings = namespace.addFeedForwardNamespace("outtake rotate storage",
            new FeedForwardSettings(0.012, 0, 0, 0.034, FeedForwardController.ControlMode.ANGULAR_POSITION));

    private final CoralJoint coralJoint;

    public RotateStorage(CoralJoint coralJoint, Supplier<Double> setpoint) {
        super(coralJoint, () -> Math.toRadians(setpoint.get()), () -> Math.toRadians(coralJoint.getPosition()),
                Storage.getInstance().hasCoral() ? outtakePIDSettings : intakePIDSettings,
                Storage.getInstance().hasCoral() ? outtakeFeedForwardSettings : intakeFeedForwardSettings);
        pidSettings.setWaitTime(() -> 0.5);
        this.coralJoint = coralJoint;
    }

    public RotateStorage(CoralJoint coralJoint, CoralJoint.StoragePose pose) {
        this(coralJoint, () -> pose.neededPitch);
    }

    @Override
    protected double calculatePIDAndFFValues() {
        if (Storage.getInstance().hasCoral()) {
            pidController.setTolerance(Math.toRadians(outtakePIDSettings.getTolerance()));
            pidController.setPID(outtakePIDSettings.getkP(), outtakePIDSettings.getkI(), outtakePIDSettings.getkD());
            pidController.setIZone(outtakePIDSettings.getIZone());
            feedForwardController.setGains(outtakeFeedForwardSettings);
        } else {
            pidController.setTolerance(Math.toRadians(intakePIDSettings.getTolerance()));
            pidController.setPID(intakePIDSettings.getkP(), intakePIDSettings.getkI(), intakePIDSettings.getkD());
            pidController.setIZone(intakePIDSettings.getIZone());
            feedForwardController.setGains(intakeFeedForwardSettings);
        }

        double pidValue = pidController.calculate(source.get(), setpoint.get());
        double svagValue = feedForwardController.calculate(source.get(), setpoint.get());
        return pidValue + svagValue;
    }

    @Override
    public boolean isFinished() {
        return !(coralJoint.canMove(coralJoint.getSpeed())) || super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
    }
}


