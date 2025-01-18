package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Storage;

import java.util.function.Supplier;

public class ReleaseCoral extends MoveGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("release corral");
    private static final Supplier<Double> TIME_TO_RELEASE = namespace.addConstantDouble("time to release", 0.5);
    private static final Supplier<Double> RELEASE_SPEED = namespace.addConstantDouble("release speed", 0.5);
    private final Storage storage;
    private double lastTimeInStorage;

    public ReleaseCoral(Storage storage) {
        super(storage, RELEASE_SPEED);
        this.storage = storage;
        this.lastTimeInStorage = 0;
    }

    @Override
    public boolean isFinished() {
        if (storage.hasCoral()) {
            lastTimeInStorage = Timer.getFPGATimestamp();
        }
        return Timer.getFPGATimestamp() - lastTimeInStorage >= TIME_TO_RELEASE;
    }
}
