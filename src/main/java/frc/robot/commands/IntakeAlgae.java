package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Gripper;
import java.util.function.Supplier;

public class IntakeAlgae extends MoveGenericSubsystem {

    private static final RootNamespace NAMESPACE = new RootNamespace("intake algae");
    private static final Supplier<Double> TIME_TO_INTAKE = NAMESPACE.addConstantDouble("time to intake", 0.5);
    private static final Supplier<Double> INTAKE_SPEED = NAMESPACE.addConstantDouble("intake speed", -0.5);

    private double startTime;

    public IntakeAlgae(Gripper gripper) {
        super(gripper, INTAKE_SPEED);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= TIME_TO_INTAKE.get();
    }
}
