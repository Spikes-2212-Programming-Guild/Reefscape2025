package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.CoralJoint;

public class RotateStorage extends MoveSmartMotorControllerGenericSubsystem {

    private static final double JOINT_SPEED = 0.3;
    private final CoralJoint coralJoint;

    public RotateStorage(CoralJoint coralJoint, PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                         CoralJoint.STORAGE_POSE pose) {
        super(coralJoint, pidSettings, feedForwardSettings, UnifiedControlMode.POSITION,
                () -> pose.neededPitch, true);
        this.coralJoint = coralJoint;
    }

    @Override
    public boolean isFinished() {
        return coralJoint.canMove(JOINT_SPEED);
    }
}
