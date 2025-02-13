package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import frc.robot.subsystems.AlgaeJoint;

import java.util.function.Supplier;

public class RotateAlgaeJointToTop extends MoveGenericSubsystem {

    private static final RootNamespace NAMESPACE = new RootNamespace("rotate algae joint to top");
    private static final Supplier<Double> SPEED = NAMESPACE.addConstantDouble("intake speed", 0.5);

    public RotateAlgaeJointToTop(AlgaeJoint algaeJoint) {
        super(algaeJoint, SPEED);
    }
}
