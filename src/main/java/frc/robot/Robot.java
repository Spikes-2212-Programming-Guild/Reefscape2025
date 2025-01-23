// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.VisionService;

import java.util.function.Supplier;

public class Robot extends TimedRobot {

    private final Namespace namespace;
    private final VisionService limelight;

    public Robot(Namespace namespace, VisionService limelight) {
        this.namespace = namespace;
        this.limelight = limelight;
    }

    @Override
    public void robotInit() {
        limelight.getRobotPose();
        namespace.putNumber("tx", limelight.getRobotPose().getX());
        namespace.putNumber("tx", limelight.getRobotPose().getRotation().getDegrees());
        namespace.putNumber("ty", limelight.getRobotPose().getY());

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
