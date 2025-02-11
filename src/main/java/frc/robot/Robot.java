// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.subsystems.AlgaeJoint;
import frc.robot.subsystems.CoralJoint;
import frc.robot.subsystems.Storage;

import java.util.function.Supplier;

public class Robot extends TimedRobot {

    private static final RootNamespace namespace = new RootNamespace("robot");
    private Storage storage;
    private AlgaeJoint algaeJoint;

    @Override
    public void robotInit() {
        storage = Storage.getInstance();
        algaeJoint = AlgaeJoint.getInstance();
        namespace.putCommand("intake coral", new IntakeCoral(storage));
        namespace.putCommand("release coral", new ReleaseCoral(storage));
        namespace.putCommand("rotate algae top", new RotateAlgaeJointToTop(algaeJoint));
        namespace.putCommand("rotate algae bottom", new RotateAlgaeJointToBottom(algaeJoint));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
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
