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
        super(elevator, pidSettings, feedForwardSettings, UnifiedControlMode.POSITION, setpoint);
        this.elevator = elevator;
    }
}
