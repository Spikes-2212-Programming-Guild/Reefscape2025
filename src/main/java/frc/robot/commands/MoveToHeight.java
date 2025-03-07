package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.Elevator;

import java.util.function.Supplier;

public class MoveToHeight extends MoveSmartMotorControllerGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("move to height");
    private static final PIDSettings pidSettings = namespace.addPIDNamespace("move to height",
            new PIDSettings(7, 0, 0, 0, 0.01, 0.5));
    private static final FeedForwardSettings feedForwardSettings = namespace.addFeedForwardNamespace("move to height",
            new FeedForwardSettings(0.4, 0, 0, 0, FeedForwardController.ControlMode.LINEAR_POSITION));

    private final Elevator elevator;

    public MoveToHeight(Elevator elevator, Supplier<Double> setpoint) {
        super(elevator, pidSettings, feedForwardSettings, UnifiedControlMode.POSITION, setpoint, false);
        this.elevator = elevator;
    }

    public MoveToHeight(Elevator elevator, Elevator.ElevatorLevel elevatorLevel) {
        this(elevator, () -> elevatorLevel.height);
    }

    @Override
    public void execute() {
        if (!isFinished()) {
            super.execute();
        }
    }

    @Override
    public boolean isFinished() {
        double error = setpoint.get() - elevator.getPosition();
        return super.isFinished() || !elevator.canMove(error);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
