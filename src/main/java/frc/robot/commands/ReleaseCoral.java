package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Storage;

public class ReleaseCoral extends MoveGenericSubsystem {

    private static final double TIME_TO_RELEASE = 0.5;
    private static final double RELEASE_SPEED = 0.5;
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
