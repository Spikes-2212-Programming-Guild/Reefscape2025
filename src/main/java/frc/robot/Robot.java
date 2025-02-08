// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Robot extends TimedRobot {

    @Override
    public void robotInit() {
        NamedCommands.registerCommand("ElevateToL4", new InstantCommand());
        NamedCommands.registerCommand("OuttakeCoralAngle", new InstantCommand());
        NamedCommands.registerCommand("ReleaseCoral", new InstantCommand());
        NamedCommands.registerCommand("ElevateToFeeder", new InstantCommand());
        NamedCommands.registerCommand("IntakeCoralAngle", new InstantCommand());
        NamedCommands.registerCommand("IntakeCoral", new InstantCommand());
        NamedCommands.registerCommand("IntakeAlgaeAngle", new InstantCommand());
        NamedCommands.registerCommand("ElevateToL3", new InstantCommand());
        NamedCommands.registerCommand("TakeAlgae", new InstantCommand());
        NamedCommands.registerCommand("PlaceAlgae", new InstantCommand());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
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
