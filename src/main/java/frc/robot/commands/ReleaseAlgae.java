package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Gripper;

import java.util.function.Supplier;


public class ReleaseAlgae extends MoveGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("release algae");
    private static final Supplier<Double> TIME_TO_RELEASE = namespace.addConstantDouble("time to release", 0);
    private static final Supplier<Double> RELEASE_SPEED = namespace.addConstantDouble("release speed", 0);

    private final Gripper gripper;

    private double lastTimeInGripper = 0;

    public ReleaseAlgae(Gripper gripper) {
        super(gripper, RELEASE_SPEED);
        this.gripper = gripper;
    }

    @Override
    public boolean isFinished() {
        if (gripper.hasAlgae()) {
            lastTimeInGripper = Timer.getFPGATimestamp();
        }
        return Timer.getFPGATimestamp() - lastTimeInGripper >= TIME_TO_RELEASE.get();
    }
}
