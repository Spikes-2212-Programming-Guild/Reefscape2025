package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Storage;

import java.util.function.Supplier;

public class ReleaseCoral extends MoveGenericSubsystem {

    private static final RootNamespace NAMESPACE = new RootNamespace("release corral");
    private static final Supplier<Double> TIME_TO_RELEASE = NAMESPACE.addConstantDouble("time to release", 0.5);
    private static final Supplier<Double> RELEASE_SPEED = NAMESPACE.addConstantDouble("release speed", 0.5);

    private final Storage storage;

    private double startTime;

    public ReleaseCoral(Storage storage) {
        super(storage, RELEASE_SPEED);
        this.storage = storage;
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= TIME_TO_RELEASE.get();
    }
}
