package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.Elevator;

public class MoveToLevel extends MoveSmartMotorControllerGenericSubsystem {

    private final Elevator elevator;
    private static final double MORE_HEIGHT = -1;

    public MoveToLevel(Elevator elevator, PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                       Elevator.ReefLevel level) {
        super(elevator, pidSettings, feedForwardSettings, UnifiedControlMode.POSITION, () -> level.height);
        this.elevator = elevator;
    }

    @Override
    public void execute() {
        subsystem.pidSet(controlMode, setpoint.get() + MORE_HEIGHT, pidSettings, feedForwardSettings);
        if (false) {

        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || elevator.canMove(elevator.getSpeed());
    }
}
