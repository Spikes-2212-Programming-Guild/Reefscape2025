package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

public class IntakeAlgae extends Command {
    private final Gripper gripper;

    public IntakeAlgae(Gripper gripper) {
        addRequirements(gripper);
        this.gripper = gripper;
    }

    @Override
    public void execute() {
        gripper.intake();
    }

    @Override
    public boolean isFinished() {
        return gripper.hasAlgae();
    }

    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }
}
