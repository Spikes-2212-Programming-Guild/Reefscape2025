package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import frc.robot.subsystems.Gripper;
import java.util.function.Supplier;

public class IntakeAlgae extends MoveGenericSubsystem {

    private static final RootNamespace NAMESPACE = new RootNamespace("intake algae");
    private static final Supplier<Double> INTAKE_SPEED = NAMESPACE.addConstantDouble("intake speed", -0.5);

    public IntakeAlgae(Gripper gripper) {
        super(gripper, INTAKE_SPEED);
    }
}
