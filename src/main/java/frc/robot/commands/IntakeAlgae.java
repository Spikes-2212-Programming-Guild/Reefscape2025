package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import frc.robot.subsystems.Gripper;

import java.util.function.Supplier;

public class IntakeAlgae extends MoveGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("intake algae");
    private static final Supplier<Double> INTAKE_SPEED = namespace.addConstantDouble("intake speed", 0);

    private final Gripper gripper;

    public IntakeAlgae(Gripper gripper) {
        super(gripper, INTAKE_SPEED);
        this.gripper = gripper;
    }

    @Override
    public boolean isFinished() {
        return gripper.hasAlgae();
    }
}
