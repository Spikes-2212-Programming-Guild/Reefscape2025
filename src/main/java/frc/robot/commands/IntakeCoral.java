package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Storage;

public class IntakeCoral extends MoveGenericSubsystem {

    private static final double INTAKE_SPEED = -0.5;
    private final Storage storage;

    public IntakeCoral(Storage storage) {
        super(storage, INTAKE_SPEED);
        addRequirements(storage);
        this.storage = storage;
    }

    @Override
    public boolean isFinished() {
        return storage.hasCoral();
    }

}
