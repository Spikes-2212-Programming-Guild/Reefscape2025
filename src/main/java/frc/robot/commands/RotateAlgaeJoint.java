package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;

import java.util.function.Supplier;

public class RotateAlgaeJoint extends MoveGenericSubsystem {

    public RotateAlgaeJoint(GenericSubsystem subsystem, Supplier<Double> speedSupplier) {
        super(subsystem, speedSupplier);
    }
}
