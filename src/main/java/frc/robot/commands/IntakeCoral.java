package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import frc.robot.subsystems.Storage;

import java.util.function.Supplier;

public class IntakeCoral extends MoveGenericSubsystem {

    private static final RootNamespace NAMESPACE = new RootNamespace("intake coral");
    private static final Supplier<Double> INTAKE_SPEED = NAMESPACE.addConstantDouble("intake speed", -0.5);

    public IntakeCoral(Storage storage) {
        super(storage, INTAKE_SPEED);
    }
}
