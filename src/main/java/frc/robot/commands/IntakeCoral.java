package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Storage;

import java.util.function.Supplier;

public class IntakeCoral extends MoveGenericSubsystem {

    private static final RootNamespace NAMESPACE = new RootNamespace("intake coral");
    private static final Supplier<Double> INTAKE_SPEED = NAMESPACE.addConstantDouble("intake speed", 0.5);
    private static final Supplier<Double> TIME_TO_INTAKE = NAMESPACE.addConstantDouble("time to intake", 0.2);

    private double startTime;

    public IntakeCoral(Storage storage) {
        super(storage, INTAKE_SPEED);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() && Timer.getFPGATimestamp() - startTime >= TIME_TO_INTAKE.get();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
