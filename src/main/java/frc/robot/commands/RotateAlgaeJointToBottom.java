package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;

import java.util.function.Supplier;

public class RotateAlgaeJointToBottom extends MoveGenericSubsystem {

    private static final RootNamespace NAMESPACE = new RootNamespace("rotate algae joint to bottom");
    private static final Supplier<Double> INTAKE_SPEED = NAMESPACE.addConstantDouble("intake speed", -0.5);

    public RotateAlgaeJointToBottom(GenericSubsystem subsystem) {
        super(subsystem, INTAKE_SPEED);
    }
}
