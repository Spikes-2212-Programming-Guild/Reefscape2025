// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.util.localization.odometry.PeriodicTaskScheduler;

public class Robot extends TimedRobot {

    private final RootNamespace namespace = new RootNamespace("robot");
    private Drivetrain drivetrain;

    @Override
    public void robotInit() {
        PeriodicTaskScheduler.init(this);
        drivetrain = Drivetrain.getInstance();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        namespace.update();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        drivetrain.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        drivetrain.resetRelativeEncoders();
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
    public void simulationPeriodic() {

    }
}
