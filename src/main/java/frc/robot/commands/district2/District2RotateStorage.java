package frc.robot.commands.district2;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystemWithPID;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.dashboard.SpikesLogger;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.district2.District2CoralJoint;

import java.util.function.Supplier;

public class District2RotateStorage extends MoveGenericSubsystemWithPID {

    private static final RootNamespace namespace = new RootNamespace("district 2 rotate storage");
    private static final PIDSettings intakePIDSettings = namespace.addPIDNamespace("intake rotate storage",
            new PIDSettings(0.2, 0.07, 0.012, 30, 4, 0.5));
    private static final FeedForwardSettings intakeFeedForwardSettings = namespace.addFeedForwardNamespace("intake rotate storage",
            new FeedForwardSettings(0, 0, 0, 0.13, FeedForwardController.ControlMode.ANGULAR_POSITION));
    private static final PIDSettings outtakePIDSettings = namespace.addPIDNamespace("outtake rotate storage",
            new PIDSettings(0.2, 0.002, 0.018, 4, 4, 0.5));
    private static final FeedForwardSettings outtakeFeedForwardSettings = namespace.addFeedForwardNamespace("outtake rotate storage",
            new FeedForwardSettings(0, 0, 0, 0.13, FeedForwardController.ControlMode.ANGULAR_POSITION));

    private final District2CoralJoint coralJoint;

    public District2RotateStorage(District2CoralJoint coralJoint, Supplier<Double> setpoint) {
        super(coralJoint, () -> Math.toRadians(setpoint.get()), () -> Math.toRadians(coralJoint.getPosition()),
                Storage.getInstance().hasCoral() ? outtakePIDSettings : intakePIDSettings,
                Storage.getInstance().hasCoral() ? outtakeFeedForwardSettings : intakeFeedForwardSettings);
        pidSettings.setWaitTime(() -> 999.0);
        this.coralJoint = coralJoint;
    }

    public District2RotateStorage(District2CoralJoint coralJoint, District2CoralJoint.StoragePose pose) {
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
}
