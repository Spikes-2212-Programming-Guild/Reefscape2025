package frc.robot.commands.district2;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystemWithPID;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import frc.robot.subsystems.district2.District2CoralJoint;

import java.util.function.Supplier;

public class District2RotateStorage extends MoveGenericSubsystemWithPID {

    private static final RootNamespace namespace = new RootNamespace("district 2 rotate storage");
    private static final PIDSettings intakePIDSettings = namespace.addPIDNamespace("rotate storage",
            new PIDSettings(0.11, 0, 0.005, 0, 3, 0.5));
    private static final FeedForwardSettings intakeFeedForwardSettings = namespace.addFeedForwardNamespace("rotate storage",
            new FeedForwardSettings(0, 0, 0, 0.082, FeedForwardController.ControlMode.ANGULAR_POSITION));
    private static final PIDSettings outtakePIDSettings = namespace.addPIDNamespace("rotate storage",
            new PIDSettings(0.11, 0, 0.005, 0, 3, 0.5));
    private static final FeedForwardSettings outtakeFeedForwardSettings = namespace.addFeedForwardNamespace("rotate storage",
            new FeedForwardSettings(0, 0, 0, 0.082, FeedForwardController.ControlMode.ANGULAR_POSITION));

    private final District2CoralJoint coralJoint;

    public District2RotateStorage(District2CoralJoint coralJoint, Supplier<Double> setpoint) {
        super(coralJoint, () -> Math.toRadians(setpoint.get()), () -> Math.toRadians(coralJoint.getPosition()),
                intakePIDSettings, intakeFeedForwardSettings);
        this.coralJoint = coralJoint;
    }

    public District2RotateStorage(District2CoralJoint coralJoint, District2CoralJoint.StoragePose pose) {
        this(coralJoint, () -> pose.neededPitch);
    }

    @Override
    public boolean isFinished() {
        return !(coralJoint.canMove(coralJoint.getSpeed())) || super.isFinished();
    }
}
