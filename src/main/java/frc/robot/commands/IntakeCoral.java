package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import frc.robot.subsystems.Storage;

import java.util.function.Supplier;

public class IntakeCoral extends MoveGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("take corral");
    private static final Supplier<Double> INTAKE_SPEED = namespace.addConstantDouble("intake speed", -0.5);
    private final Storage storage;

    public IntakeCoral(Storage storage) {
        super(storage, INTAKE_SPEED);
        this.storage = storage;
    }
}
