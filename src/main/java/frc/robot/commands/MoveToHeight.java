package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.
        MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.UnifiedControlMode;
import frc.robot.subsystems.Elevator;

import java.util.function.Supplier;

public class MoveToHeight extends MoveSmartMotorControllerGenericSubsystem {

    private final Elevator elevator;

    public MoveToHeight(Elevator elevator, PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                        Supplier<Double> setpoint) {
        super(elevator, pidSettings, feedForwardSettings, UnifiedControlMode.POSITION, setpoint, true);
        this.elevator = elevator;
    }

    public MoveToHeight(Elevator elevator, PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                        Elevator.ElevatorLevel elevatorLevel) {
        this(elevator, pidSettings, feedForwardSettings, () -> elevatorLevel.height);
    }

    @Override
    public void execute() {
        if (!isFinished())
            super.execute();
    }

    @Override
    public boolean isFinished() {
        double error = setpoint.get() - elevator.getPosition();
        return super.isFinished() || !elevator.canMove(error);
    }
}
