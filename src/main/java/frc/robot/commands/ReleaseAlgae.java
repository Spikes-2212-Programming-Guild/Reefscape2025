package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveArcadeWithPID;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Gripper;

public class ReleaseAlgae extends MoveGenericSubsystem {

    private static final double TIME_TO_RELEASE = 0.5;
    private static final double RELEASE_SPEED = 0.5;

    private final Gripper gripper;

    private double lastTimeInGripper = 0;

    public ReleaseAlgae(Gripper gripper) {
        super(gripper, RELEASE_SPEED);
        addRequirements(gripper);
        this.gripper = gripper;
    }

    @Override
    public boolean isFinished() {
        if (gripper.hasAlgae()) {
            lastTimeInGripper = Timer.getFPGATimestamp();
        }
        return Timer.getFPGATimestamp() - lastTimeInGripper >= TIME_TO_RELEASE;
    }
}
