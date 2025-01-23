package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.
        MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.CoralJoint;

public class RotateStorage extends MoveSmartMotorControllerGenericSubsystem {

    public RotateStorage(CoralJoint coralJoint, PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                         CoralJoint.STORAGE_POSE pose) {
        super(coralJoint, pidSettings, feedForwardSettings, UnifiedControlMode.POSITION, ()-> pose.degree);
    }
}
