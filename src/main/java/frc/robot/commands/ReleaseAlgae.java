package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

public class ReleaseAlgae extends Command {

    private static final double TIME_TO_RELEASE = 0.5;
    private final Gripper gripper;
    private double lastTimeInGripper;

    public ReleaseAlgae(Gripper gripper) {
        this.gripper = gripper;
        this.lastTimeInGripper = 0;
    }

    @Override
    public void execute() {
        gripper.release();
    }

    @Override
    public boolean isFinished() {
        if (gripper.hasAlgae()) {
            lastTimeInGripper = Timer.getFPGATimestamp();
        }
        return Timer.getFPGATimestamp() - lastTimeInGripper >= TIME_TO_RELEASE;
    }

    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }
}
