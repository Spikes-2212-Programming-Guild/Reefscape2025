package frc.robot.commands.district2;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.CoralJoint;
import frc.robot.subsystems.district2.District2CoralJoint;

import java.util.function.Supplier;

public class District2RotateStorage extends MoveSmartMotorControllerGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("district 2 rotate storage");
    private static final PIDSettings pidSettings = namespace.addPIDNamespace("rotate storage",
            new PIDSettings(1.3, 1.5, 0.13, 0, 0.02, 999));
    private static final FeedForwardSettings feedForwardSettings = namespace.addFeedForwardNamespace("rotate storage",
            new FeedForwardSettings(0, 0, 0, 0.05, FeedForwardController.ControlMode.ANGULAR_POSITION));

    private final District2CoralJoint coralJoint;

    public District2RotateStorage(District2CoralJoint coralJoint, Supplier<Double> setpoint) {
        super(coralJoint, pidSettings, feedForwardSettings, UnifiedControlMode.POSITION,
                setpoint, false);
        this.coralJoint = coralJoint;
    }

    public District2RotateStorage(District2CoralJoint coralJoint, District2CoralJoint.StoragePose pose) {
        this(coralJoint, () -> pose.neededPitch);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return !(coralJoint.canMove(coralJoint.getSpeed())) || super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}

