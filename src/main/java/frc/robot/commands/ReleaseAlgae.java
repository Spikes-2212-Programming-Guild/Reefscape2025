package frc.robot.commands;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Gripper;
import java.util.function.Supplier;

public class ReleaseAlgae extends MoveGenericSubsystem {

    private static final RootNamespace NAMESPACE = new RootNamespace("release algae");
    private static final Supplier<Double> TIME_TO_RELEASE = NAMESPACE.addConstantDouble("time to release", 0.5);
    private static final Supplier<Double> RELEASE_SPEED = NAMESPACE.addConstantDouble("release speed", 0.5);

    private final Gripper gripper;

    private double startTime;

    public ReleaseAlgae(Gripper gripper) {
        super(gripper, RELEASE_SPEED);
        this.gripper = gripper;
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= TIME_TO_RELEASE.get() && !gripper.hasAlgae();
    }
}
