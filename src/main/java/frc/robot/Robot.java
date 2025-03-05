// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.NamedCommands;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.JustDrive;
import frc.robot.commands.district2.District2RotateStorage;
import frc.robot.subsystems.*;
import frc.robot.subsystems.district2.District2CoralJoint;

public class Robot extends TimedRobot {

    RootNamespace namespace = new RootNamespace("robot");
    private Drivetrain drivetrain;
    private Elevator elevator;
    private Storage storage;
    private Gripper gripper;
    private AlgaeJoint algaeJoint;
    private District2CoralJoint coralJoint;
    OI oi = new OI();

    @Override
    public void robotInit() {
        drivetrain = Drivetrain.getInstance();
        namespace.putNumber("left x", oi::getLeftX);
        namespace.putNumber("left y", oi::getLeftY);
        namespace.putNumber("right x", oi::getRightX);
        namespace.putNumber("right y", oi::getRightY);
        algaeJoint = AlgaeJoint.getInstance();
        getInstances();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        coralJoint.calibrateEncoderPosition();
//        elevator.calibratePosition();
        namespace.update();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        drivetrain.setNeutralMode(NeutralModeValue.Brake);
        coralJoint.brake();
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
        drivetrain.resetGyro();
        drivetrain.resetRelativeEncoders();
        JustDrive auto = new JustDrive(drivetrain);
//        DriveAndPlaceL2 auto = new DriveAndPlaceL2(drivetrain, coralJoint, storage);
        auto.schedule();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        coralJoint.coast();
        drivetrain.resetRelativeEncoders();
        drivetrain.setNeutralMode(NeutralModeValue.Coast);
        drivetrain.setDefaultCommand(new Drive(drivetrain, () -> oi.getLeftY() * 4, () -> oi.getLeftX() * 4, () -> oi.getRightX() * 6,
                true, false, false));
//        algaeJoint.setDefaultCommand(new MoveGenericSubsystem(algaeJoint, AlgaeJoint.STABILIZATION_SPEED).onlyIf(gripper::hasAlgae));
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

    private void registerNamedCommands() {
        NamedCommands.registerCommand("OuttakeCoralAngleL2",
                new District2RotateStorage(coralJoint, District2CoralJoint.StoragePose.L2));
        NamedCommands.registerCommand("LimelightRight", new CenterOnReef(drivetrain, OI.Side.RIGHT));
        NamedCommands.registerCommand("LimelightLeft", new CenterOnReef(drivetrain, OI.Side.LEFT));
        NamedCommands.registerCommand("LimelightFeeder", new CenterOnFeeder(drivetrain));
        NamedCommands.registerCommand("ReleaseCoral", new ReleaseCoral(storage));
        NamedCommands.registerCommand("IntakeCoralAngle",
                new District2RotateStorage(coralJoint, District2CoralJoint.StoragePose.INTAKE));
        NamedCommands.registerCommand("IntakeCoral", new IntakeCoral(storage));
//        NamedCommands.registerCommand("OuttakeCoralAngle",
//                new RotateStorage(coralJoint, CoralJoint.StoragePose.PLACEMENT));
        //        NamedCommands.registerCommand("ElevateToFeeder", new MoveToHeight(elevator, Elevator.ElevatorLevel.FEEDER));
//        NamedCommands.registerCommand("IntakeAlgaeAngle", new RotateAlgaeJointToBottom(algaeJoint));
//        NamedCommands.registerCommand("ElevateToL3", new MoveToHeight(elevator, Elevator.ElevatorLevel.L3));
//        NamedCommands.registerCommand("TakeAlgae", new IntakeAlgae(gripper));
//        NamedCommands.registerCommand("PlaceAlgae", new ReleaseAlgae(gripper));
//        NamedCommands.registerCommand("ElevateToL4", new MoveToHeight(elevator, Elevator.ElevatorLevel.L4));
    }

    private void getInstances() {
        elevator = Elevator.getInstance();
        storage = Storage.getInstance();
        gripper = Gripper.getInstance();
        coralJoint = District2CoralJoint.getInstance();
        algaeJoint = AlgaeJoint.getInstance();
    }
}
