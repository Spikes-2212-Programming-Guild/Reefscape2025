package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.CoralJoint;

import java.util.function.Supplier;

public class RotateStorage extends MoveSmartMotorControllerGenericSubsystem {

    private final CoralJoint coralJoint;

    public RotateStorage(CoralJoint coralJoint, PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                         Supplier<Double> setpoint) {
        super(coralJoint, pidSettings, feedForwardSettings, UnifiedControlMode.POSITION,
                setpoint, true);
        this.coralJoint = coralJoint;
    }

    public RotateStorage(CoralJoint coralJoint, PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                         CoralJoint.STORAGE_POSE pose) {
        this(coralJoint, pidSettings, feedForwardSettings, () -> pose.neededPitch);
    }

    @Override
    public boolean isFinished() {
        return !(coralJoint.canMove(coralJoint.getSpeed())) || super.isFinished();
    }
}
