// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {

    private final Elevator elevator = Elevator.getInstance();
    private final Storage storage = Storage.getInstance();
    private final Gripper gripper = Gripper.getInstance();
    private final CoralJoint coralJoint = CoralJoint.getInstance();
    private final AlgaeJoint algaeJoint = AlgaeJoint.getInstance();

    @Override
    public void robotInit() {
        NamedCommands.registerCommand("ElevateToL4", new MoveToHeight(elevator, Elevator.ElevatorLevel.L4));
        NamedCommands.registerCommand("OuttakeCoralAngle",
                new RotateStorage(coralJoint, CoralJoint.StoragePose.PLACEMENT));
        NamedCommands.registerCommand("ReleaseCoral", new ReleaseCoral(storage));
        NamedCommands.registerCommand("ElevateToFeeder", new MoveToHeight(elevator, Elevator.ElevatorLevel.FEEDER));
        NamedCommands.registerCommand("IntakeCoralAngle", new RotateStorage(coralJoint, CoralJoint.StoragePose.INTAKE));
        NamedCommands.registerCommand("IntakeCoral", new IntakeCoral(storage));
        NamedCommands.registerCommand("IntakeAlgaeAngle", new RotateAlgaeJointToBottom(algaeJoint));
        NamedCommands.registerCommand("ElevateToL3", new MoveToHeight(elevator, Elevator.ElevatorLevel.L3));
        NamedCommands.registerCommand("TakeAlgae", new IntakeAlgae(gripper));
        NamedCommands.registerCommand("PlaceAlgae", new ReleaseAlgae(gripper));
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
