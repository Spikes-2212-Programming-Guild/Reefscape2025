package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.CoralJoint;

import java.util.function.Supplier;

public class RotateStorage extends MoveSmartMotorControllerGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("rotate storage");
    private static final PIDSettings pidSettings = namespace.addPIDNamespace("rotate storage");
    private static final FeedForwardSettings feedForwardSettings = namespace.addFeedForwardNamespace("rotate storage",
            FeedForwardController.ControlMode.LINEAR_POSITION);

    private final CoralJoint coralJoint;

    public RotateStorage(CoralJoint coralJoint, Supplier<Double> setpoint) {
        super(coralJoint, pidSettings, feedForwardSettings, UnifiedControlMode.POSITION,
                setpoint, true);
        this.coralJoint = coralJoint;
    }

    public RotateStorage(CoralJoint coralJoint, CoralJoint.STORAGE_POSE pose) {
        this(coralJoint, () -> pose.neededPitch);
    }

    @Override
    public boolean isFinished() {
        return !(coralJoint.canMove(coralJoint.getSpeed())) || super.isFinished();
    }
}
