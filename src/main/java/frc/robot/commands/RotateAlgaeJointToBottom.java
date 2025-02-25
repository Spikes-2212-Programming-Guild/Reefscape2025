package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.AlgaeJoint;

import java.util.function.Supplier;

public class RotateAlgaeJointToBottom extends MoveGenericSubsystem {

    private static final RootNamespace NAMESPACE = new RootNamespace("rotate algae joint to bottom");
    private static final Supplier<Double> SPEED = NAMESPACE.addConstantDouble("intake speed", -0.5);
    private static final Supplier<Double> TIME_TO_RELEASE = NAMESPACE.addConstantDouble("time to release", 0.5);

    private double startTime;

    public RotateAlgaeJointToBottom(AlgaeJoint algaeJoint) {
        super(algaeJoint, SPEED);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() && Timer.getFPGATimestamp() - startTime >= TIME_TO_RELEASE.get();
    }
}
