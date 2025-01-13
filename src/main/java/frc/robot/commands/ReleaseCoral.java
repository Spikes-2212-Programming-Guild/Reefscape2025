package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Storage;

public class ReleaseCoral extends Command {

    private static final double TIME_TO_RELEASE = 0.5;
    private final Storage storage;
    private double lastTimeInStorage;

    public ReleaseCoral(Storage storage) {
        addRequirements(storage);
        this.storage = storage;
        this.lastTimeInStorage = 0;
    }

    @Override
    public void execute() {
        storage.release();
    }

    @Override
    public boolean isFinished() {
        if (storage.hasCoral()) {
            lastTimeInStorage = Timer.getFPGATimestamp();
        }
        return Timer.getFPGATimestamp() - lastTimeInStorage >= TIME_TO_RELEASE;
    }

    @Override
    public void end(boolean interrupted) {
        storage.stop();
    }
}
