package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Storage;

public class IntakeCoral extends Command {

    private final Storage storage;

    public IntakeCoral(Storage storage) {
        addRequirements(storage);
        this.storage = storage;
    }

    @Override
    public void execute() {
        storage.intake();
    }

    @Override
    public boolean isFinished() {
        return storage.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        storage.stop();
    }
}
