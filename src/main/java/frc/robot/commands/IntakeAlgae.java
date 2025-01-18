package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.Gripper;

public class IntakeAlgae extends MoveGenericSubsystem {

    private static final double INTAKE_SPEED = -0.5;

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
