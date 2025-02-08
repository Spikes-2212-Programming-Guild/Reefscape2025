// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.MoveToHeight;
import frc.robot.subsystems.Elevator;

import java.util.function.Supplier;

public class Robot extends TimedRobot {

    private static final RootNamespace namespace = new RootNamespace("elevator");
    private final PIDSettings pidSettings = namespace.addPIDNamespace("elevator");
    private final FeedForwardSettings feedForwardSettings = namespace.addFeedForwardNamespace("elevator",
            FeedForwardController.ControlMode.LINEAR_POSITION);
    private final Supplier<Double> setpoint = namespace.addConstantDouble("setpoint", 0);
    private final Elevator elevator = Elevator.getInstance();
    private final MoveToHeight moveToHeight = new MoveToHeight(elevator, pidSettings, feedForwardSettings, setpoint);

    @Override
    public void robotInit() {

        namespace.putCommand("move to height", moveToHeight);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        elevator.calibratePosition();
        namespace.update();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
